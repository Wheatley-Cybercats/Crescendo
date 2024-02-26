package frc.robot.Commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;


public class AutoAimCommand extends Command {
    double speakerHeight = 0; //all in meters
    double distanceFromSpeaker;
    double shooterAngle;
    Pose2d currentPose;
    Transform2d difference;

   /* private final LeadScrew leadscrew = Robot.leadscrew;


    public AutoAimCommand(LeadScrew leadscrew) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.leadscrew);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        currentPose = new Pose2d(
                Robot.limelight.getBOTPOSE_WPIRED()[0],
                Robot.limelight.getBOTPOSE_WPIRED()[1], //TODO: when this works, make this based on which alliance we're on
                Rotation2d.fromDegrees(Robot.limelight.getBOTPOSE_WPIRED()[5])
        );

        difference = targetPose.minus(currentPose);
        double xdistance = difference.getX();
        double ydistance = difference.getY();

        distanceFromSpeaker = Math.sqrt(xdistance*xdistance + ydistance*ydistance);
        shooterAngle = Math.atan2(speakerHeight, distanceFromSpeaker);



        //TODO: sets angle of shooter based on angleOfShooter



    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    */


}
