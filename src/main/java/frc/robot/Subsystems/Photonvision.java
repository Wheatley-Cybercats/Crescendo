package frc.robot.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class Photonvision {
    PhotonCamera camera1;
    PhotonCamera camera2;
    Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    Transform3d robotToCam2 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();



    public Photonvision(){
        camera1 = new PhotonCamera("Camera1");
        camera2 = new PhotonCamera("Camera2");
    }

    public PhotonPipelineResult getResult1(){
        return camera1.getLatestResult();
    }

    public PhotonPipelineResult getResult2(){
        return camera2.getLatestResult();
    }

    public Pose2d getPoseBLUE(){
        PhotonPipelineResult result1 = getResult1();
        PhotonPipelineResult result2 = getResult2();
        Transform3d FTC1 = new Transform3d();
        Transform3d FTC2 = new Transform3d();
        if (result1.getMultiTagResult().estimatedPose.isPresent) {
            FTC1 = result1.getMultiTagResult().estimatedPose.best;
        }
        if (result2.getMultiTagResult().estimatedPose.isPresent) {
            FTC2 = result2.getMultiTagResult().estimatedPose.best;
        }

        return new Pose2d(
                (FTC1.getX()+FTC2.getX())/2,
                (FTC1.getY()+FTC2.getY())/2,
                new Rotation2d(
                        (FTC1.getRotation().toRotation2d().getDegrees() + FTC1.getRotation().toRotation2d().getDegrees())/2
                ));
    }

}
