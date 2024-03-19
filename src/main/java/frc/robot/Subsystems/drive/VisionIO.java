// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public Pose2d pose = new Pose2d();
    public Pose2d pose_wpiBlue = new Pose2d();
    public Pose2d pose_wpiRed = new Pose2d();
    public boolean hasTarget = false;
    public Pose3d pose3d = new Pose3d();
    public double tagArea = 0;
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

  public default Pose3d getBotPose3d() {
    return new Pose3d();
  }

  public default double getTimeStamp() {
    return 0;
  }
  ;

  public default boolean getHasTarget() {
    return false;
  }

  public default double getTagArea() {
    return 0;
  }

  public default void updateIOInputs(VisionIOInputs inputs) {}
  ;
}
