// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public Pose2d visionPose = new Pose2d();
    public double visionTimestampPhoton = 0;
    public boolean hasNote = false;
    public double tagArea = 0;
    public double noteTX = 0;
    public double noteTY = 0;
  }

  public default boolean isConnected() {
    return false;
  }

  public default Pose2d getVisionPose() {
    return new Pose2d();
  }

  public default boolean hasNote() {
    return false;
  }

  public default double getTagArea() {
    return 0;
  }

  public default double getNoteTX() {
    return 0;
  }

  public default double getNoteTY() {
    return 0;
  }

  public default double getVisionTimestampPhoton() {
    return 0;
  }

  public default void updateIOInputs(VisionIOInputs inputs) {}
}
