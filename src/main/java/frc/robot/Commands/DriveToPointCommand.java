package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Photonvision;
import frc.robot.Subsystems.Vision;
import org.photonvision.PhotonUtils;

import static frc.robot.Robot.gyroPIDController;
import static frc.team2872.HelperFunctions.Normalize_Gryo_Value;


public class DriveToPointCommand extends Command {
    Vision vision = Robot.vision;

    private final Pose2d targetPose;
    private Pose2d currentPose;

    public DriveToPointCommand(Pose2d target) {
        this.targetPose = target;
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentPose = vision.getPose();

        Transform2d dif = targetPose.minus(currentPose);

        double driveAngle = RobotProperties.FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(Math.atan2(dif.getY(), dif.getX()) - Robot.gyro.getAngle()) : Math.atan2(dif.getY(), dif.getX());

        SmartDashboard.putNumber("Drive x", dif.getY());
        SmartDashboard.putNumber("Drive y", dif.getY());
        SmartDashboard.putNumber("Drive rot", dif.getRotation().getDegrees());

        gyroPIDController.disablePID();
        Robot.swerveDrive.drive(driveAngle, Math.hypot(dif.getX(), dif.getY()), dif.getRotation().getDegrees(), true);


    }

    @Override
    public boolean isFinished() {
        return ((targetPose.getX() - currentPose.getX()) < 0.02) && ((targetPose.getY() - currentPose.getY()) < 0.02) && ((targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()) < 5);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.drive(0,0,0,false);
    }

    /* good:
        currentPose = new Pose2d(

                ll.getBOTPOSE_WPIRED()[0],
                ll.getBOTPOSE_WPIRED()[1],
                Rotation2d.fromDegrees(ll.getBOTPOSE_WPIRED()[5])
        );


        Transform2d dif = targetPose.minus(currentPose);

         */
        /*
        double driveAngle = RobotProperties.FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(Math.atan2(drive_xyr[1], drive_xyr[0]) - Robot.gyro.getAngle()) : Math.atan2(drive_xyr[1], drive_xyr[0]);
        double driveMagnitude = Math.min(Math.sqrt(drive_xyr[0]*drive_xyr[0] + drive_xyr[1]*drive_xyr[1]), 0.2);
        double rotationMagnitude = 0;
        if(drive_xyr[2] > 0){
            rotationMagnitude = Math.min(Math.sqrt(drive_xyr[2]), 0.2);
        }else if(drive_xyr[2] < 0){
            rotationMagnitude = Math.min(Math.sqrt(Math.abs(drive_xyr[2])), 0.2);
            rotationMagnitude -= 2 * rotationMagnitude;
        }
        SmartDashboard.putNumberArray("Drive mag", new double[] {driveAngle, driveMagnitude, rotationMagnitude});

         */
        /*
        double driveAngle = RobotProperties.FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(Math.atan2(dif.getY(), dif.getX()) - Robot.gyro.getAngle()) : Math.atan2(dif.getY(), dif.getX());

        SmartDashboard.putNumber("Drive x", dif.getY());
        SmartDashboard.putNumber("Drive y", dif.getY());
        SmartDashboard.putNumber("Drive rot", dif.getRotation().getDegrees());

        gyroPIDController.disablePID();
        Robot.swerveDrive.drive(driveAngle, Math.hypot(dif.getX(), dif.getY()), dif.getRotation().getDegrees(), true);

         */


}

