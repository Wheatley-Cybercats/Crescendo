// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** Add your docs here. */
public class VisionIOPhotonLight implements VisionIO {
  private PhotonCamera leftCam = new PhotonCamera("leftCam");
  private PhotonCamera rightCam = new PhotonCamera("rightCam");
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  PhotonPoseEstimator leftCamPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          leftCam,
          Constants.leftCamToRobot);
  PhotonPoseEstimator rightCamPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          rightCam,
          Constants.rightCamToRobot);

  @Override
  public boolean isConnected() {
    return leftCam.isConnected() && rightCam.isConnected() && nt.getInstance().isConnected();
  }

  @Override
  public Pose2d getVisionPose() {
    leftCamPoseEstimator.setReferencePose(RobotState.getInstance().getEstimatedPose());
    rightCamPoseEstimator.setReferencePose(RobotState.getInstance().getEstimatedPose());
    Pose2d pose;
    if (leftCamPoseEstimator.update().isPresent() && rightCamPoseEstimator.update().isPresent()) {
      Pose2d leftEstimatedPose = leftCamPoseEstimator.update().get().estimatedPose.toPose2d();
      Pose2d rightEstimatedPose = rightCamPoseEstimator.update().get().estimatedPose.toPose2d();
      double x = (leftEstimatedPose.getX() + rightEstimatedPose.getX()) / 2;
      double y = (leftEstimatedPose.getY() + rightEstimatedPose.getY()) / 2;
      Rotation2d rotation =
          Rotation2d.fromDegrees(
              (leftEstimatedPose.getRotation().getDegrees()
                      + rightEstimatedPose.getRotation().getDegrees())
                  / 2);
      pose = new Pose2d(x, y, rotation);
    } else if (leftCamPoseEstimator.update().isPresent()
        && rightCamPoseEstimator.update().isEmpty()) {
      pose = leftCamPoseEstimator.update().get().estimatedPose.toPose2d();
    } else if (leftCamPoseEstimator.update().isEmpty()
        && rightCamPoseEstimator.update().isPresent()) {
      pose = rightCamPoseEstimator.update().get().estimatedPose.toPose2d();
    } else {
      pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }
    return pose;
  }

  @Override
  public double getVisionTimestampPhoton() {
    return (leftCam.getLatestResult().getTimestampSeconds()
            + rightCam.getLatestResult().getTimestampSeconds())
        / 2;
  }

  @Override
  public double getTagArea() {
    if (leftCam.getLatestResult().hasTargets() && rightCam.getLatestResult().hasTargets()) {
      return (leftCam.getLatestResult().getBestTarget().getArea()
              + rightCam.getLatestResult().getBestTarget().getArea())
          / 2;
    } else if (leftCam.getLatestResult().hasTargets() && !rightCam.getLatestResult().hasTargets()) {
      return leftCam.getLatestResult().getBestTarget().getArea();
    } else if (!leftCam.getLatestResult().hasTargets() && rightCam.getLatestResult().hasTargets()) {
      return rightCam.getLatestResult().getBestTarget().getArea();
    } else {
      return 0;
    }
  }

  @Override
  public boolean hasNote() {
    return nt.getEntry("tv").getDouble(0) == 1;
  }

  @Override
  public double getNoteTX() {
    return nt.getEntry("tx").getDouble(0);
  }

  @Override
  public double getNoteTY() {
    return nt.getEntry("ty").getDouble(0);
  }

  @Override
  public void updateIOInputs(VisionIOInputs input) {
    input.connected = isConnected();
    input.visionPose = getVisionPose();
    input.hasNote = hasNote();
    input.tagArea = getTagArea();
    input.noteTX = getNoteTX();
    input.noteTY = getNoteTY();
  }
}
