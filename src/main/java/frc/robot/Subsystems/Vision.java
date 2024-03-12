package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

import java.util.Arrays;

public class Vision {
    LimeLight limeLight = Robot.limelight;
    Photonvision photon = Robot.photon;

    /**
     * Returns the pose
     *
     * @return
     * The robot position relative to WPI_BLUE based on the average of all cameras (2 photon, 1 LimeLight)
     */
    public Pose2d getPose(){
        Pose2d LLPose = new Pose2d(

                limeLight.getBOTPOSE_WPIBLUE()[0],
                limeLight.getBOTPOSE_WPIBLUE()[1],
                Rotation2d.fromDegrees(limeLight.getBOTPOSE_WPIBLUE()[5])
        );

        Pose2d photonPose = photon.getPoseBLUE();

        return filterAndAveragePoses(LLPose, photonPose);

    }

    /**
     * Used to combine Pose2Ds
     *
     * @param poses
     * Varargs amount of Pose2D
     *
     * @return
     * The average Pose2D values between the given
     */
    public static Pose2d filterAndAveragePoses(Pose2d... poses) {
        //find pose
        double meanX = Arrays.stream(poses).mapToDouble(Pose2d::getX).average().orElse(0);
        double meanY = Arrays.stream(poses).mapToDouble(Pose2d::getY).average().orElse(0);
        double meanRotation = Arrays.stream(poses).mapToDouble(pose -> pose.getRotation().getRadians()).average().orElse(0);

        //find standard deviation for x, y, and rotation
        double stdDevX = Math.sqrt(Arrays.stream(poses).mapToDouble(pose -> Math.pow(pose.getX() - meanX, 2)).average().orElse(0));
        double stdDevY = Math.sqrt(Arrays.stream(poses).mapToDouble(pose -> Math.pow(pose.getY() - meanY, 2)).average().orElse(0));
        double stdDevRotation = Math.sqrt(Arrays.stream(poses).mapToDouble(pose -> Math.pow(pose.getRotation().getRadians() - meanRotation, 2)).average().orElse(0));

        //filter out outliers
        Pose2d[] filteredPoses = Arrays.stream(poses)
                .filter(pose -> Math.abs(pose.getX() - meanX) <= 2 * stdDevX &&
                        Math.abs(pose.getY() - meanY) <= 2 * stdDevY &&
                        Math.abs(pose.getRotation().getRadians() - meanRotation) <= 2 * stdDevRotation)
                .toArray(Pose2d[]::new);

        //get average of filtered poses
        double avgX = Arrays.stream(filteredPoses).mapToDouble(Pose2d::getX).average().orElse(0);
        double avgY = Arrays.stream(filteredPoses).mapToDouble(Pose2d::getY).average().orElse(0);
        double avgRotation = Arrays.stream(filteredPoses).mapToDouble(pose -> pose.getRotation().getRadians()).average().orElse(0);

        return new Pose2d(avgX, avgY, new Rotation2d(avgRotation));
    }

    /**
     * Converts a Transform3d object to a Pose2d object.
     *
     * @param transform3d The 3D transformation to convert.
     * @return A Pose2d object representing the 2D position and orientation.
     */
    public static Pose2d transform3dToPose2d(Transform3d transform3d) {
        // Extract the X and Y translations
        double x = transform3d.getTranslation().getX();
        double y = transform3d.getTranslation().getX();

        // Extract the yaw rotation. This method assumes your Transform3d class can give you the yaw directly.
        // You may need to convert from another representation of 3D rotation.
        double yawRadians = transform3d.getRotation().getAngle();

        // Create a new Pose2d object with the extracted position and rotation
        return new Pose2d(x, y, new Rotation2d(yawRadians));
    }
}
