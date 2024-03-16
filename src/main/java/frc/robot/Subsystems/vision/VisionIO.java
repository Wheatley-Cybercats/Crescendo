// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface VisionIO {
  public default void setPipeline(int pipeline) {}

  public default void setCamMode(int mode) {}

  public default Pose2d getBotPose() {
    return new Pose2d();
  }

  public default Pose2d getBotPose_WPIRED() {
    return new Pose2d();
  }

  public default Pose2d getBotPose_WPIBLUE() {
    return new Pose2d();
  }
}
