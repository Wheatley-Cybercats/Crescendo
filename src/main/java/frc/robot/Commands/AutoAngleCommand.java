package frc.robot.Commands;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import static frc.robot.RobotProperties.blueSpeakerPose;
import static frc.robot.RobotProperties.redSpeakerPose;
import frc.robot.Subsystems.LeadScrew;


public class AutoAngleCommand extends Command {
    double speakerHeight = 0; //all in meters
    double distanceFromSpeaker;
    double shooterAngle;
    Pose2d currentPose;
    Pose2d speakerPose;
    Transform2d difference;
    boolean isRedAlliance = false;
    double desired;

    private final LeadScrew leadscrew = Robot.leadscrew;


    public AutoAngleCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.leadscrew);
    }

    @Override
    public void initialize() {

         if (DriverStationJNI.getAllianceStation() == AllianceStationID.Red1 || DriverStationJNI.getAllianceStation() == AllianceStationID.Red2 || DriverStationJNI.getAllianceStation() == AllianceStationID.Red3) {
            isRedAlliance = true;
            speakerPose = redSpeakerPose;
        }
        else if (DriverStationJNI.getAllianceStation() == AllianceStationID.Blue1 || DriverStationJNI.getAllianceStation() == AllianceStationID.Blue2 || DriverStationJNI.getAllianceStation() == AllianceStationID.Blue3){
            isRedAlliance = false;
            speakerPose= blueSpeakerPose;
        }

        currentPose = new Pose2d(
                Robot.limelight.getBOTPOSE_WPIBLUE()[0],
                Robot.limelight.getBOTPOSE_WPIBLUE()[1],
                Rotation2d.fromDegrees(Robot.limelight.getBOTPOSE_WPIBLUE()[5])
        );
    }

    @Override
    public void execute() {

        //difference = speakerPose.minus(currentPose);
        double xdistance = speakerPose.getX() - currentPose.getX();
        double ydistance = speakerPose.getY() - currentPose.getY(); //x and y on the ground

        distanceFromSpeaker = Math.sqrt(xdistance*xdistance + ydistance*ydistance);
        //shooterAngle = Math.atan2(speakerHeight, distanceFromSpeaker);
        desired = regression(distanceFromSpeaker);
        leadscrew.moveToPosition(regression(desired));


    }

    @Override
    public boolean isFinished() {
        return leadscrew.atPosition(desired);
    }

    @Override
    public void end(boolean interrupted) {
        leadscrew.stop();
    }

    private double regression(double x){
        double a = 197.523;
        double b = 0.578254;
        return Math.pow(b, x) * a;
    }



}
