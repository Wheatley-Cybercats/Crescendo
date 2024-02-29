package frc.robot.Commands;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.Robot.*;
import static frc.robot.RobotProperties.blueSpeakerPose;
import static frc.robot.RobotProperties.redSpeakerPose;
import static frc.team2872.HelperFunctions.Normalize_Gryo_Value;

public class AlignHorizontallyCommand extends Command {
    Pose2d currentPose;
    Pose2d speakerPose;
    boolean alliancePresent = DriverStation.getAlliance().isPresent();
    boolean allianceIsRed = String.valueOf(DriverStation.getAlliance()).equals("Optional[Red]");
    boolean allianceIsBlue = String.valueOf(DriverStation.getAlliance()).equals("Optional[Blue]");
    boolean isRedAlliance = false;
    Transform2d difference;
    double adjustedAngle;

    public AlignHorizontallyCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
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

        currentPose = vision.getPose();

        SmartDashboard.putNumber("limgeis", limelight.getBOTPOSE_WPIBLUE()[0]);

    }

    @Override
    public void execute() {

        //TODO: when on red alliance, gyro and limelight WPIBLUE are negative inverses.
        // when on blue alliance, gyro and limelight WPIBLUE are 180 opposite

        SmartDashboard.putString("DS alliance", String.valueOf(DriverStation.getAlliance()));
        SmartDashboard.putNumber("speakerPoseX" , speakerPose.getX());
        SmartDashboard.putNumber("speakerPoseY" , speakerPose.getY());
        SmartDashboard.putNumber("currPoseX" , currentPose.getX());
        SmartDashboard.putNumber("currPoseY" , currentPose.getY());


        difference = speakerPose.minus(currentPose);
        //difference = new Transform2d(currentPose, speakerPose); //speakerPose is nonexistent because it was not defined in init
        double xdistance = speakerPose.getX() - currentPose.getX();
        double ydistance = speakerPose.getY() - currentPose.getY();
        SmartDashboard.putNumber("xdist", xdistance);
        SmartDashboard.putNumber("ydist", ydistance);

        double angleFromPerpendicular = Math.toDegrees(Math.atan2(ydistance, xdistance));
        SmartDashboard.putNumber("angleFromPerpendicular", angleFromPerpendicular);

        if (isRedAlliance) adjustedAngle = -angleFromPerpendicular;
        if (!isRedAlliance) adjustedAngle = Normalize_Gryo_Value(angleFromPerpendicular + 180); //angleFromPerpendicular but rotated by 180

        if (vision.getPose().getX() != 0 && (Math.abs(gyro.getAngle() - angleFromPerpendicular) > 4)){
            gyroPIDController.updateSensorLockValue(adjustedAngle);
            Robot.quickTurning = true;
            Robot.swerveDrive.drive(0, 0, gyroPIDController.getPIDValue(), false);
        }




        /* no longer used
        Pose2d currentPose = new Pose2d(
                Math.round(Robot.limelight.getBOTPOSE_WPIRED()[0]),
                Math.round(Robot.limelight.getBOTPOSE_WPIRED()[1]),
                Rotation2d.fromDegrees(Math.round(Robot.limelight.getBOTPOSE_WPIRED()[5]))
        );

        angleOffset = currentPose.getRotation().minus(Rotation2d.fromDegrees(180)).getDegrees();
        SmartDashboard.putNumber("hi", angleOffset);

        if (angleOffset < -2 || angleOffset > 2){
            Robot.swerveDrive.drive(0, 0, angleOffset < 0 ? -Math.sqrt(Math.abs(angleOffset*.005))*1.3 : Math.sqrt(angleOffset*.005)*1.3, true);
        } else{
            Robot.swerveDrive.drive(0,0,0,false);
        }
         */
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
