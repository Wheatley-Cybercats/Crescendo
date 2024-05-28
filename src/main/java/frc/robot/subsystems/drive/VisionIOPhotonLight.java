// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOPhotonLight implements VisionIO {
  private PhotonCamera leftCam = new PhotonCamera("leftCam");
  private PhotonCamera rightCam = new PhotonCamera("rightCam");
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public boolean isConnected() {
    return leftCam.isConnected() && rightCam.isConnected() && nt.getInstance().isConnected();
  }

  @Override
  public Pose2d getVisionPose() {
    var resultLeftCam = leftCam.getLatestResult();
    Transform3d fieldToLeftCam = new Transform3d();
    if (resultLeftCam.getMultiTagResult().estimatedPose.isPresent) {
      fieldToLeftCam = resultLeftCam.getMultiTagResult().estimatedPose.best;
    }
    var resultRightCam = rightCam.getLatestResult();
    Transform3d fieldToRightCam = new Transform3d();
    if (resultRightCam.getMultiTagResult().estimatedPose.isPresent) {
      fieldToRightCam = resultRightCam.getMultiTagResult().estimatedPose.best;
    }

    fieldToLeftCam.plus(Constants.leftCamToRobot);
    fieldToRightCam.plus(Constants.rightCamToRobot);

    double x =
        (fieldToLeftCam.getTranslation().getX() + fieldToRightCam.getTranslation().getX()) / 2;
    double y =
        (fieldToLeftCam.getTranslation().getY() + fieldToRightCam.getTranslation().getY()) / 2;
    Rotation2d rotation =
        Rotation2d.fromDegrees(
            (fieldToLeftCam.getRotation().toRotation2d().getDegrees()
                    + fieldToRightCam.getRotation().toRotation2d().getDegrees())
                / 2);
    return new Pose2d(x, y, rotation);
  }

  @Override
  public double getVisionTimestampPhoton() {
    return (leftCam.getLatestResult().getTimestampSeconds()
            + rightCam.getLatestResult().getTimestampSeconds())
        / 2;
  }

  @Override
  public double getTagArea() {
    return nt.getEntry("ta").getDouble(0);
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
