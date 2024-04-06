package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class VisionIOPhoton implements VisionIO{

    PhotonCamera photonCamera;


    public VisionIOPhoton(String camName){
        photonCamera = new PhotonCamera(camName);
    }

    @Override
    public void setPipeline(int pipeline){
        DriverStation.reportWarning("Pipeline cannot be set through code for PhotonVision", false);
    }

    @Override
    public void setCamMode(int mode){
        DriverStation.reportWarning("Cam mode cannot be set through code for PhotonVision", false);
    }

    @Override
    public Pose2d getBotPose(){
        var result = photonCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        
    }
}
