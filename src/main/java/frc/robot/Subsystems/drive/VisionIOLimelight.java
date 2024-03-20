// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {

  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void setPipeline(int pipeline) {
    nt.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void setCamMode(int mode) {
    nt.getEntry("camMode").setNumber(mode);
  }

  @Override
  public Pose2d getBotPose() {
    var table = nt.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose2d(
        table[0],
        table[1],
        new Rotation2d(Math.toRadians(table[5])));
  }

  @Override
  public Pose2d getBotPose_WPIRED() {
    var table = nt.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose2d(
        table[0],
        table[1],
            new Rotation2d(Math.toRadians(table[5])));
  }

  @Override
  public Pose2d getBotPose_WPIBLUE() {
    var table = nt.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose2d(
        table[0],
        table[1],
            new Rotation2d(Math.toRadians(table[5])));
  }

  @Override
  public Pose3d getBotPose3d() {
    var table = nt.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(
        table[0],
        table[1],
        table[2],
        new Rotation3d(
            Math.toRadians(table[3]),
            Math.toRadians(table[4]),
            Math.toRadians(table[5])));
  }

  @Override
  public double getTimeStamp() {
    return nt.getEntry("botpose").getDoubleArray(new double[7])[6]; //why is it length 7 ?????
  }

  @Override
  public double getTagArea() {
    return nt.getEntry("ta").getDouble(0);
  }

  @Override
  public boolean getHasTarget() {
    return nt.getEntry("tv").getDouble(0) == 1;
  }

  @Override
  public void updateIOInputs(VisionIOInputs input) {
    input.connected = getBotPose().getX() >= 0;
    input.pose = getBotPose();
    input.pose_wpiBlue = getBotPose_WPIBLUE();
    input.pose_wpiRed = getBotPose_WPIRED();
    input.pose3d = getBotPose3d();
    if (nt.getEntry("tv").getDouble(0) == 1) {
      input.hasTarget = true;
    } else {
      input.hasTarget = false;
    }
    input.tagArea = nt.getEntry("ta").getDouble(0);
  }
}
