/*
 * Copyright 2018 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.ar.core.examples.java.augmentedimage.rendering;

import android.content.Context;
import com.google.ar.core.Anchor;
import com.google.ar.core.AugmentedImage;
import com.google.ar.core.Pose;
import com.google.ar.core.examples.java.common.rendering.ObjectRenderer;
import com.google.ar.core.examples.java.common.rendering.ObjectRenderer.BlendMode;
import java.io.IOException;

/** Renders an augmented image. */
public class AugmentedImageRenderer {
  private static final String TAG = "AugmentedImageRenderer";

  private static final float TINT_INTENSITY = 0.1f;
  private static final float TINT_ALPHA = 1.0f;
  private static final int[] TINT_COLORS_HEX = {
    0x000000, 0xF44336, 0xE91E63, 0x9C27B0, 0x673AB7, 0x3F51B5, 0x2196F3, 0x03A9F4, 0x00BCD4,
    0x009688, 0x4CAF50, 0x8BC34A, 0xCDDC39, 0xFFEB3B, 0xFFC107, 0xFF9800,
  };

  private final ObjectRenderer imageFrameUpperLeft = new ObjectRenderer();
  private final ObjectRenderer imageFrameUpperRight = new ObjectRenderer();
  private final ObjectRenderer imageFrameLowerLeft = new ObjectRenderer();
  private final ObjectRenderer imageFrameLowerRight = new ObjectRenderer();

  private final ObjectRenderer mazeRenderer = new ObjectRenderer();
  private final ObjectRenderer andyRenderer = new ObjectRenderer();

  // Create a new pose for the Andy
  private Pose andyPose = Pose.IDENTITY;

  public AugmentedImageRenderer() {}

  public void createOnGlThread(Context context) throws IOException {

    mazeRenderer.createOnGlThread(
        context, "models/green-maze/GreenMaze.obj", "models/frame_base.png");
    mazeRenderer.setMaterialProperties(0.0f, 3.5f, 1.0f, 6.0f);

    andyRenderer.createOnGlThread(
        context, "models/andy.obj", "models/andy.png");
    andyRenderer.setMaterialProperties(0.0f, 3.5f, 1.0f, 6.0f);
  }

  public void draw(
      float[] viewMatrix,
      float[] projectionMatrix,
      AugmentedImage augmentedImage,
      Anchor centerAnchor,
      float[] colorCorrectionRgba) {
    float[] tintColor =
        convertHexToColor(TINT_COLORS_HEX[augmentedImage.getIndex() % TINT_COLORS_HEX.length]);

    final float mazeEdgeSize = 492.65f; // Magic number of maze size
    final float maxImageEdgeSize = Math.max(augmentedImage.getExtentX(), augmentedImage.getExtentZ()); // Get largest detected image edge size

    Pose anchorPose = centerAnchor.getPose();

    float mazeScaleFactor = maxImageEdgeSize / mazeEdgeSize; // scale to set Maze to image size
    float[] modelMatrix = new float[16];

    // OpenGL Matrix operation is in the order: Scale, rotation and Translation
    // So the manual adjustment is after scale
    // The 251.3f and 129.0f is magic number from the maze obj file
    // You mustWe need to do this adjustment because the maze OBJobj file
    // is not centered around origin. Normally when you
    // work with your own model, you don't have this problem.
    Pose mazeModelLocalOffset = Pose.makeTranslation(
        -251.3f * mazeScaleFactor,
        0.0f,
        129.0f * mazeScaleFactor);
    anchorPose.compose(mazeModelLocalOffset).toMatrix(modelMatrix, 0);
    mazeRenderer.updateModelMatrix(modelMatrix, mazeScaleFactor, mazeScaleFactor/10.0f, mazeScaleFactor); // This line relies on a change in ObjectRenderer.updateModelMatrix later in this codelab.
    mazeRenderer.draw(viewMatrix, projectionMatrix, colorCorrectionRgba, tintColor);

    // Adjust the Andy's rendering position
    // The Andy's pose is at the maze's vertex's coordinate
    Pose andyPoseInImageSpace = Pose.makeTranslation(
        andyPose.tx() * mazeScaleFactor,
        andyPose.ty() * mazeScaleFactor,
        andyPose.tz() * mazeScaleFactor);

    anchorPose.compose(andyPoseInImageSpace).toMatrix(modelMatrix, 0);
    andyRenderer.updateModelMatrix(modelMatrix, 0.05f);
    andyRenderer.draw(viewMatrix, projectionMatrix, colorCorrectionRgba, tintColor);

  }

  public void updateAndyPose(Pose pose) {
    andyPose = pose;
  }

  private static float[] convertHexToColor(int colorHex) {
    // colorHex is in 0xRRGGBB format
    float red = ((colorHex & 0xFF0000) >> 16) / 255.0f * TINT_INTENSITY;
    float green = ((colorHex & 0x00FF00) >> 8) / 255.0f * TINT_INTENSITY;
    float blue = (colorHex & 0x0000FF) / 255.0f * TINT_INTENSITY;
    return new float[] {red, green, blue, TINT_ALPHA};
  }
}
