/*
 * Copyright 2018 Google LLC
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

package com.google.ar.sceneform.samples.augmentedimage;

import android.content.Context;
import android.net.Uri;
import android.util.Log;
import com.google.ar.core.AugmentedImage;
import com.google.ar.core.Pose;
import com.google.ar.sceneform.AnchorNode;
import com.google.ar.sceneform.Node;
import com.google.ar.sceneform.math.Quaternion;
import com.google.ar.sceneform.math.Vector3;
import com.google.ar.sceneform.rendering.Color;
import com.google.ar.sceneform.rendering.MaterialFactory;
import com.google.ar.sceneform.rendering.ModelRenderable;
import com.google.ar.sceneform.rendering.Renderable;
import com.google.ar.sceneform.rendering.ShapeFactory;
import com.google.ar.schemas.lull.Quat;

import java.util.concurrent.CompletableFuture;

/**
 * Node for rendering an augmented image. The image is framed by placing the virtual picture frame
 * at the corners of the augmented image trackable.
 */
@SuppressWarnings({"AndroidApiChecker"})
public class AugmentedImageNode extends AnchorNode {

  private static final String TAG = "AugmentedImageNode";

  // The augmented image represented by this node.
  private AugmentedImage image;
  private Node mazeNode;
  private Node ballNode;

  private CompletableFuture<ModelRenderable> mazeRenderable;
  private ModelRenderable ballRenderable;
  private float maze_scale = 0.0f;

  public AugmentedImageNode(Context context) {
    mazeRenderable =
          ModelRenderable.builder()
              .setSource(context, Uri.parse("GreenMaze.sfb"))
              .build();

    MaterialFactory.makeOpaqueWithColor(context, new Color(android.graphics.Color.RED))
        .thenAccept(
            material -> {
              ballRenderable =
                  ShapeFactory.makeSphere(0.01f, new Vector3(0, 0, 0), material); });
  }

  /**
   * Called when the AugmentedImage is detected and should be rendered. A Sceneform node tree is
   * created based on an Anchor created from the image. The corners are then positioned based on the
   * extents of the image. There is no need to worry about world coordinates since everything is
   * relative to the center of the image, which is the parent node of the corners.
   */
  @SuppressWarnings({"AndroidApiChecker", "FutureReturnValueIgnored"})
  public void setImage(AugmentedImage image) {
    this.image = image;

    // If any of the models are not loaded, then recurse when all are loaded.
    if (!mazeRenderable.isDone()) {
      Log.d(TAG, "loading maze renderable still in progress. Wait to render again");
      CompletableFuture.allOf(mazeRenderable)
          .thenAccept((Void aVoid) -> setImage(image))
          .exceptionally(
              throwable -> {
                Log.e(TAG, "Exception loading", throwable);
                return null;
              });
      return;
    }

    // Set the anchor based on the center of the image.
    setAnchor(image.createAnchor(image.getCenterPose()));

    mazeNode = new Node();
    mazeNode.setParent(this);
    mazeNode.setRenderable(mazeRenderable.getNow(null));

    // Make sure longest edge fits inside the image
    final float maze_edge_size = 492.65f;
    final float max_image_edge = Math.max(image.getExtentX(), image.getExtentZ());
    maze_scale = max_image_edge / maze_edge_size;

    // Scale Y extra 10 times to lower the wall of maze
    mazeNode.setLocalScale(new Vector3(maze_scale, maze_scale * 0.1f, maze_scale));

    // Add the ball
    ballNode = new Node();
    ballNode.setParent(this);
    ballNode.setRenderable(ballRenderable);
    ballNode.setLocalPosition(new Vector3(0, 0.1f, 0)); // start pos for debugging

    // Expected maze mesh size of ball(in original mesh vertices) is 13 diagram, 6.5 radius.
    // when mesh is scaled down to max_image_edge, radius will be scaled down to 6.5 * scale.
    // The sphere is already 0.01, so we need to scale the ball ball_scale = 6.5 * scale / 0.01f
    ballNode.setLocalScale(new Vector3(
        6.5f * maze_scale / 0.01f,
        6.5f * maze_scale / 0.01f,
        6.5f * maze_scale / 0.01f));
  }

  public void updateBallPose(Pose pose) {
    if (ballNode == null)
      return;

    ballNode.setLocalPosition(new Vector3(pose.tx() * maze_scale, pose.ty()* maze_scale, pose.tz()* maze_scale));
    ballNode.setLocalRotation(new Quaternion(pose.qx(), pose.qy(), pose.qz(), pose.qw()));
  }
  
  public AugmentedImage getImage() {
    return image;
  }
}
