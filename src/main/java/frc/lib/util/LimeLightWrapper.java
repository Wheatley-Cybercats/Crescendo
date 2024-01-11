

package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.Optional;

/**
 * PhotonCamera-based Pose Estimator.

 */
public class LimeLightWrapper {
    private NetworkTable table;

    public LimeLightWrapper() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Gets if photonvision can see a target.
     */
    public boolean seesTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1;
    }

    /**
     * Get estimated pose without a prior.
     *
     * @return an estimated Pose2d based solely on apriltags
     */
    public Pose2d getPose() {
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5]));
    }

    public double latency() {
        return table.getEntry("cl").getDouble(0) + table.getEntry("tl").getDouble(0);
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public SwerveDrivePoseEstimator getEstimatedGlobalPose(SwerveDrivePoseEstimator prevEstimatedRobotPose) {
        prevEstimatedRobotPose.addVisionMeasurement(getPose(), latency());
        return prevEstimatedRobotPose;
    }
}
