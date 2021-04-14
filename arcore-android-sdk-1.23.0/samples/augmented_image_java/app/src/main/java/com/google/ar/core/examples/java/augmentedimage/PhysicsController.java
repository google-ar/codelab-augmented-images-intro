/*
 * Copyright 2019 Google LLC All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.core.examples.java.augmentedimage;

import android.content.Context;
import android.util.Log;

import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.IndexedMesh;
import com.bulletphysics.collision.shapes.ScaledBvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.google.ar.core.Pose;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import de.javagl.obj.Obj;
import de.javagl.obj.ObjData;
import de.javagl.obj.ObjReader;
import de.javagl.obj.ObjUtils;

import static com.bulletphysics.collision.dispatch.CollisionObject.DISABLE_DEACTIVATION;


public class PhysicsController {

  private static final String TAG = "PhysicsController";

  private DiscreteDynamicsWorld dynamicsWorld;
  private SequentialImpulseConstraintSolver solver;
  private RigidBody ballRB;
  private long previous_time;
  private Context context;
  private final float MAZE_SCALE = 0.02f;
  private final float MAZE_SCALE_Y_EXTRA = 0.1f;

  public PhysicsController(Context activity) {
    context = activity;
    initialize();
  }

  public void initialize() {
    // Initialize Bullet Physics Engine
    DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
    CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
    DbvtBroadphase broadphase = new DbvtBroadphase();
    solver = new SequentialImpulseConstraintSolver();

    dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    // Override default gravity, which would be (0, -10, 0)
    // Because I want to manually control the gravity based on maze's Pose
    dynamicsWorld.setGravity(new Vector3f(0f, 0f, 0f));

    AddMazeRigidBody();
    AddBallRigidBody();

    previous_time = java.lang.System.currentTimeMillis();
  }

  private void AddBallRigidBody() {
    // Read comments in AddMazeRigidBody to see why we choose 0.13 for Ball's radius
    CollisionShape ballShape = new SphereShape(0.13f);

    Transform ballTransform = new Transform();
    ballTransform.setIdentity();
    ballTransform.origin.set(0, 0.5f, 0); // Slightly raise the ball at the beginning

    DefaultMotionState ballMotionState = new DefaultMotionState(ballTransform);
    RigidBodyConstructionInfo ballRBInfo = new RigidBodyConstructionInfo(
        0.2f, ballMotionState, ballShape, new Vector3f(0, 0, 0));

    ballRB = new RigidBody(ballRBInfo);
    ballRB.setActivationState(DISABLE_DEACTIVATION);

    dynamicsWorld.addRigidBody(ballRB);
  }

  //
  // Load GreenMaze.obj and uses the vertices info inside as mesh for the maze
  // Because GreenMaze.obj doesn't have a bottom, so it won't catch the ball
  // We'll add a plane of infinite size at Y=0, as bottom.
  //
  private void AddMazeRigidBody() {
    IndexedMesh mazeMesh = loadMazeMesh();
    if (mazeMesh == null)
      return;

    //
    // Maze rigid body has two parts
    // 1. The maze mesh, centered around origin
    // 2. A plane, at origin, as bottom of maze
    //
    // For the maze mesh, we'll use static mesh shape
    // This is the way to work with concave shape mesh in jBullet.
    // In jBullet, static mesh cannot move, so I'll only move the ball.
    //
    // Here're two special design in Physics calculation
    // 1) All things are in "maze" 's coordinate space.
    //    - To do so, Maze is centered at origin. bottom at Y=0.
    //    - When we add force to ball at runtime, we also convert gravity to Maze's space.
    // 2) All size are a good size for jBullet's simulation.
    //    Which means take tips from here https://facultyfp.salisbury.edu/despickler/personal/Resources/GraphicsExampleCodeGLSL_SFML/InterfaceDoc/Bullet/Bullet_User_Manual.pdf
    //    Particularly, avoid very small and very large shape => Keep triangles in [0.2, 10] size range.
    //    In order to do so, we'll scale the maze to 0.02 scale on X, Y, Z, and then additional 0.1 on Y.
    //      (Why Y is 0.002 instead of 0.02? Because we want to lower the height of maze)
    // 3) Reduce Maze wall height, to 1/10 it's original height (Slightly higher than radius of ball)
    //    , so we can see ball when the ball reaches the bottom.
    //
    // Facts of the maze mesh:
    //   - The maze's original size is (492, 120, 492) in (x, y, z) order.
    //   - The maze's gap allows movement of a ball at about 13 diameter.
    // After scale,
    //   - the maze's size in physics simulation is (9.84, 0.24, 9.84)
    //   - which allows ball of 0.26 diameter (0.13 radius)
    //

    // Maze part 1, triangle mesh
    TriangleIndexVertexArray mazeMashVertexArray = new TriangleIndexVertexArray();
    mazeMashVertexArray.addIndexedMesh(mazeMesh);

    BvhTriangleMeshShape mazeShape = new BvhTriangleMeshShape(mazeMashVertexArray, false);
    ScaledBvhTriangleMeshShape scaledMazeShape = new ScaledBvhTriangleMeshShape(
        mazeShape, new Vector3f(MAZE_SCALE, MAZE_SCALE * MAZE_SCALE_Y_EXTRA, MAZE_SCALE));

    // For static concave mesh shape, Transform has to be identity
    Transform mazeTransform = new Transform();
    mazeTransform.setIdentity();

    DefaultMotionState mazeMotionState = new DefaultMotionState(mazeTransform);
    RigidBodyConstructionInfo mazeRBInfo = new RigidBodyConstructionInfo(
        0, mazeMotionState, scaledMazeShape, new Vector3f(0, 0, 0));
    mazeRBInfo.friction = 0.1f;
    RigidBody mazeRB = new RigidBody(mazeRBInfo);
    dynamicsWorld.addRigidBody(mazeRB); // add the body to the dynamics world


    // Maze part 2, plane as bottom
    CollisionShape fakeGroundShape = new StaticPlaneShape(
        new Vector3f(0, 1.0f, 0),
        0);

    Transform fakeGroundTransform = new Transform();
    fakeGroundTransform.setIdentity();
    fakeGroundTransform.origin.set(0, 0, 0);

    DefaultMotionState fakeGroundMotionState = new DefaultMotionState(fakeGroundTransform);
    RigidBodyConstructionInfo fakeGroundRBInfo = new RigidBodyConstructionInfo(
        0.0f, fakeGroundMotionState, fakeGroundShape, new Vector3f(0, 0, 0));
    fakeGroundRBInfo.friction = 0.1f;
    RigidBody fakeGroundRB = new RigidBody(fakeGroundRBInfo);
    dynamicsWorld.addRigidBody(fakeGroundRB);
  }

  private IndexedMesh loadMazeMesh() {
    IndexedMesh indexedMesh = null;

    try {
      InputStream objInputStream = context.getAssets().open("GreenMaze.obj");
      Obj obj = ObjReader.read(objInputStream);
      obj = ObjUtils.convertToRenderable(obj);

      IntBuffer wideIndices = ObjData.getFaceVertexIndices(obj, 3);
      FloatBuffer vertices = ObjData.getVertices(obj);

      // Usage reference:
      // https://github.com/LapisSea/OpenGL-Bullet-engine/blob/08848ea0e42c08cb43d5bb2fd3722cc1d1fa0b80/src/com/lapissea/opengl/util/UtilM.java
      indexedMesh = new IndexedMesh();
      indexedMesh.numTriangles = wideIndices.limit() / 3;
      indexedMesh.triangleIndexBase = ByteBuffer.allocateDirect(wideIndices.limit()*4).order(ByteOrder.nativeOrder());
      indexedMesh.triangleIndexBase.asIntBuffer().put(wideIndices);
      indexedMesh.triangleIndexStride = 3 * 4;
      indexedMesh.numVertices = vertices.limit();
      indexedMesh.vertexBase = ByteBuffer.allocateDirect(vertices.limit()*4).order(ByteOrder.nativeOrder());

      //
      // Get the mesh min, max and center, so we can recenter vertices
      //
      float minX = Float.MAX_VALUE;
      float minY = Float.MAX_VALUE;
      float minZ = Float.MAX_VALUE;
      float maxX = -Float.MAX_VALUE;
      float maxY = -Float.MAX_VALUE;
      float maxZ = -Float.MAX_VALUE;

      for (int i = 0; i < vertices.limit(); i = i + 3) {
        float x = vertices.get(i);
        float y = vertices.get(i+1);
        float z = vertices.get(i+2);
        minX = Float.min(x, minX);
        minY = Float.min(y, minY);
        minZ = Float.min(z, minZ);
        maxX = Float.max(x, maxX);
        maxY = Float.max(y, maxY);
        maxZ = Float.max(z, maxZ);
      }

      float centerX = (minX + maxX) / 2.0f;
      float centerZ = (minZ + maxZ) / 2.0f;

      //
      // Re-center vertices, put them into indexedMesh
      //
      FloatBuffer fbVertex = indexedMesh.vertexBase.asFloatBuffer();
      for (int i = 0; i < vertices.limit(); i = i + 3) {
        float x = vertices.get(i);
        float y = vertices.get(i+1);
        float z = vertices.get(i+2);
        x = x - centerX; // x at [-1/2 * sizeX, 1/2 * sizeX]
        y = y - minY;    // y at [0, sizeY]
        z = z - centerZ; // z at [-1/2 * sizeZ, 1/2 * sizeZ]

        fbVertex.put(x);
        fbVertex.put(y);
        fbVertex.put(z);
      }

      indexedMesh.vertexStride = 3 * 4;
    } catch (IOException e) {
      Log.e(TAG, "Failed to read an asset file GreenMaze.obj", e);
    }

    return indexedMesh;
  }


  public void updatePhysics() {
    long current_time = java.lang.System.currentTimeMillis();

    // stepSimulation takes deltaTime in the unit of seconds
    dynamicsWorld.stepSimulation((current_time - previous_time) / 1000.0f);
    previous_time = current_time;

    //printDebugInfo();
  }

  private void printDebugInfo() {
    //
    // Help print out debug info
    //
    int numObj = dynamicsWorld.getNumCollisionObjects();
    ObjectArrayList<CollisionObject> objArray = dynamicsWorld.getCollisionObjectArray();
    for (int j = 0; j < numObj; ++j) {
      CollisionObject collisionObj = objArray.get(j);
      RigidBody body = RigidBody.upcast(collisionObj);
      Transform worldTransform = new Transform();
      int state = collisionObj.getActivationState();
      if (body != null && body.getMotionState() != null) {
        body.getMotionState().getWorldTransform(worldTransform);
      } else {
        collisionObj.getWorldTransform(worldTransform);
      }

      Log.d(TAG,
          String.format("obj %d status [%d] World transform %f, %f, %f",
              j, state,
              worldTransform.origin.x, worldTransform.origin.y, worldTransform.origin.z));
    }
  }

  // Get the pose on Ball, in Maze's coordinate space
  // - With centered the vertices in Maze, (0, 0, 0) is the center of the maze
  // - Though we scaled maze in physics simulation, the Pose returned here is not scaled.
  public Pose getBallPose() {
    Transform ballTransform = new Transform();
    ballRB.getMotionState().getWorldTransform(ballTransform);

    Quat4f rot = new Quat4f();
    ballTransform.getRotation(rot);

    // Use MAZE_SCALE to convert size from physical world size to Maze's original size
    // Because in display size, Sceneform is actually dealing with original size of Maze
    float translation[] = {ballTransform.origin.x / MAZE_SCALE, ballTransform.origin.y / MAZE_SCALE, ballTransform.origin.z/ MAZE_SCALE};
    float rotation[] = {rot.x, rot.y, rot.z, rot.w};

    Pose ballPose = new Pose(translation, rotation);
    return ballPose;
  }

  public void applyGravityToBall(float[] mazeGravity) {
//    Log.d(TAG,
//        String.format("Apply force to ball %f, %f, %f",
//            mazeGravity[0], mazeGravity [1], mazeGravity[2]));

    ballRB.applyCentralForce(new Vector3f(mazeGravity));
  }
}
