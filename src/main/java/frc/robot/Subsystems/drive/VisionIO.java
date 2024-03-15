// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public Pose2d pose = new Pose2d();
    public Pose2d pose_wpiBlue = new Pose2d();
    public Pose2d pose_wpiRed = new Pose2d();
  }

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

  public default double getTimeStamp() {
    return 0;
  }
  ;

  public default void updateIOInputs(VisionIOInputs inputs) {}
  ;
}
