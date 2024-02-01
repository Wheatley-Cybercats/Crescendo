package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.LimeLight;

import static frc.robot.Robot.gyroPIDController;
import static frc.team2872.HelperFunctions.Normalize_Gryo_Value;


public class DriveToPointCommand extends Command {

    private final Pose2d target;
    private Pose2d cur;

    LimeLight ll = new LimeLight();

    public DriveToPointCommand(Pose2d target) {
        this.target = target;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        /*
        if(RobotProperties.USE_ODOMETRY_DRIVE_TO_POINT) {
            cur = new Pose2d(
                    (ll.getBOTPOSE()[0] + Robot.swerveDrive.getPose().getX()) / 2,
                    ll.getBOTPOSE()[1] + Robot.swerveDrive.getPose().getY() / 2,
                    new Rotation2d((ll.getBOTPOSE()[5] + Robot.swerveDrive.getPose().getRotation().getDegrees()) / 2)
            );
        }else{

         */
            cur = new Pose2d(
                    ll.getBOTPOSE()[0],
                    ll.getBOTPOSE()[1],
                    new Rotation2d(ll.getBOTPOSE()[5])
            );
        //}

        double[] drive_xyr = new double[3];
        drive_xyr[0] = Math.min(0.5, Math.sqrt(target.getX()-cur.getX()));
        drive_xyr[1] = Math.min(0.5, Math.sqrt(target.getY()-cur.getY()));
        drive_xyr[2] = Math.sqrt(target.getRotation().getDegrees()-cur.getRotation().getDegrees());
        double fieldCorrectedAngle = RobotProperties.FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(Math.atan2(drive_xyr[1], drive_xyr[0]) - Robot.gyro.getAngle()) : Math.atan2(drive_xyr[1], drive_xyr[0]);
        gyroPIDController.disablePID();
        Robot.swerveDrive.drive(fieldCorrectedAngle, Math.min(0.5, Math.sqrt(Math.sqrt(drive_xyr[0]*drive_xyr[0] + drive_xyr[1]*drive_xyr[1]))), drive_xyr[2], false);
    }

    @Override
    public boolean isFinished() {
        return ((target.getX() - cur.getX()) < 0.02) && ((target.getY() - cur.getY()) < 0.02) && ((target.getRotation().getDegrees() - cur.getRotation().getDegrees()) < 5);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
