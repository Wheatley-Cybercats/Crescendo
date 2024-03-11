package frc.team2872.drive;

// Java Imports

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.team2872.sensors.UDPClient;

import java.io.*;
import java.util.LinkedList;
import java.util.List;

import static frc.team2872.HelperFunctions.*;

/**
 * @author Mark Ebert
 */
public class SwerveDrive implements RobotProperties {

    // Swerve Drive Units
    private SwerveUnit leftRearUnit, leftFrontUnit, rightFrontUnit, rightRearUnit;

    // Global Variables
    private volatile double leftRearAngle = 0, leftFrontAngle = 0, rightFrontAngle = 0, rightRearAngle = 0;

    // PID Data Logger
    private List<UDPClient> udpClients;
    private SwerveDriveKinematics swerveDriveKinematics;

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public SwerveDrive(final SwerveUnitConfig lrUnitConfig, final SwerveUnitConfig lfUnitConfig, final SwerveUnitConfig rfUnitConfig,
            final SwerveUnitConfig rrUnitConfig) {
        // Init Swerve Units
        this.leftRearUnit = new SwerveUnit(lrUnitConfig);
        this.leftFrontUnit = new SwerveUnit(lfUnitConfig);
        this.rightFrontUnit = new SwerveUnit(rfUnitConfig);
        this.rightRearUnit = new SwerveUnit(rrUnitConfig);

        // Setup PID logging, if enabled in swerve unit config
        try {
            udpClients = new LinkedList<UDPClient>();
            udpClients.add(new UDPClient(leftRearUnit.getSlewPIDData(), PID_LOG_ADDRESS, 5801));
            udpClients.add(new UDPClient(leftFrontUnit.getSlewPIDData(), PID_LOG_ADDRESS, 5802));
            udpClients.add(new UDPClient(rightFrontUnit.getSlewPIDData(), PID_LOG_ADDRESS, 5803));
            udpClients.add(new UDPClient(rightRearUnit.getSlewPIDData(), PID_LOG_ADDRESS, 5804));

            udpClients.forEach(client -> {
                client.start();
            });
        } catch (IOException ex) {
            // Do Nothing
            System.out.println("Could not start any PID logging.");
            // ex.printStackTrace();
        }

        // Load the save slew calibrations
        loadSlewCalibration();

        swerveDriveKinematics = new SwerveDriveKinematics(
                lfUnitConfig.getMODULE_LOCATION(),
                rfUnitConfig.getMODULE_LOCATION(),
                lrUnitConfig.getMODULE_LOCATION(),
                rrUnitConfig.getMODULE_LOCATION());

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics, Robot.gyro.getRotation2d(),
                new SwerveModulePosition[]{
                        leftFrontUnit.getSwerveModulePosition(),
                        rightFrontUnit.getSwerveModulePosition(),
                        leftRearUnit.getSwerveModulePosition(),
                        rightRearUnit.getSwerveModulePosition()
                }, new Pose2d(new Translation2d(Robot.limelight.getBOTPOSE()[0], Robot.limelight.getBOTPOSE()[1]), new Rotation2d(Robot.limelight.getBOTPOSE()[5]))
        );

    }

    public void driveInit() {
        leftRearAngle = 0;
        leftFrontAngle = 0;
        rightFrontAngle = 0;
        rightRearAngle = 0;
        leftRearUnit.enable();
        leftFrontUnit.enable();
        rightFrontUnit.enable();
        rightRearUnit.enable();
    }

    public void drive(final double driveAngle, final double driveMagnitude, final double rotationalMagnitude, final boolean precisionMode) {

        SmartDashboard.putNumber("driveAngle", driveAngle);
        SmartDashboard.putNumber("driveMagnitude", driveMagnitude);
        SmartDashboard.putNumber("rotationalMagitude", rotationalMagnitude);

        double leftFrontMagnitude, leftRearMagnitude, rightFrontMagnitude, rightRearMagnitude;
        if (driveMagnitude != 0 || rotationalMagnitude != 0) {
            // Create the initial vector
            final double[] driveVector = new double[] { driveAngle, driveMagnitude };

            // Perform the vector addition to get the resultant vectors
            final double[] leftFrontResultantVector = Add_Two_Vectors(driveVector, new double[] { 45, rotationalMagnitude });//correct original: 45 || what works for us: -45
            final double[] leftRearResultantVector = Add_Two_Vectors(driveVector, new double[] { -45, rotationalMagnitude });//-45 || -135
            final double[] rightFrontResultantVector = Add_Two_Vectors(driveVector, new double[] { 135, rotationalMagnitude });//135 || 45
            final double[] rightRearResultantVector = Add_Two_Vectors(driveVector, new double[] { -135, rotationalMagnitude });//-135 || 135

            // Update the angles
            leftFrontAngle = leftFrontResultantVector[0];
            leftRearAngle = leftRearResultantVector[0];
            rightFrontAngle = rightFrontResultantVector[0];
            rightRearAngle = rightRearResultantVector[0];

            // Find vector with the largest magnitude
            final double[] largestVector = Return_Vector_With_Largest_Magnitude(leftFrontResultantVector, leftRearResultantVector, rightFrontResultantVector,
                    rightRearResultantVector);

            // Update the magnitudes, if the largest vector exceeds 1.0 then scale all others relative to it
            leftFrontMagnitude = Map(leftFrontResultantVector[1], 0, largestVector[1] > 1 ? largestVector[1] : 1, 0, precisionMode ?  0.2 : MAX_DRIVE_SPEED);
            leftRearMagnitude = Map(leftRearResultantVector[1], 0, largestVector[1] > 1 ? largestVector[1] : 1, 0, precisionMode ?  0.2 : MAX_DRIVE_SPEED);
            rightFrontMagnitude = Map(rightFrontResultantVector[1], 0, largestVector[1] > 1 ? largestVector[1] : 1, 0, precisionMode ?  0.2 : MAX_DRIVE_SPEED);
            rightRearMagnitude = Map(rightRearResultantVector[1], 0, largestVector[1] > 1 ? largestVector[1] : 1, 0, precisionMode  ?  0.2 : MAX_DRIVE_SPEED);
        } else {
            // Keep the current wheel angles but update the magnitudes
            leftFrontMagnitude = 0;
            leftRearMagnitude = 0;
            rightFrontMagnitude = 0;
            rightRearMagnitude = 0;
        }

        // Swerve Direction Optimization
        if (SWERVE_UNIT_ORIENTATION_OPTIMIZATION) {
            final double lfAngleDisplacement = Math.abs(Get_Gyro_Displacement(leftFrontUnit.getSlewAngle(), leftFrontAngle));
            final double lrAngleDisplacement = Math.abs(Get_Gyro_Displacement(leftRearUnit.getSlewAngle(), leftRearAngle));
            final double rfAngleDisplacement = Math.abs(Get_Gyro_Displacement(rightFrontUnit.getSlewAngle(), rightFrontAngle));
            final double rrAngleDisplacement = Math.abs(Get_Gyro_Displacement(rightRearUnit.getSlewAngle(), rightRearAngle));

            if (lfAngleDisplacement > 90) {
                leftFrontAngle = Normalize_Gryo_Value(leftFrontAngle + 180);
                leftFrontMagnitude = -leftFrontMagnitude;
            }
            if (lrAngleDisplacement > 90) {
                leftRearAngle = Normalize_Gryo_Value(leftRearAngle + 180);
                leftRearMagnitude = -leftRearMagnitude;
            }
            if (rfAngleDisplacement > 90) {
                rightFrontAngle = Normalize_Gryo_Value(rightFrontAngle + 180);
                rightFrontMagnitude = -rightFrontMagnitude;
            }
            if (rrAngleDisplacement > 90) {
                rightRearAngle = Normalize_Gryo_Value(rightRearAngle + 180);
                rightRearMagnitude = -rightRearMagnitude;
            }
        }

        // Updates the slew motor angles
        leftFrontUnit.updateSlewAngle(leftFrontAngle);
        leftRearUnit.updateSlewAngle(leftRearAngle);
        rightFrontUnit.updateSlewAngle(rightFrontAngle);
        rightRearUnit.updateSlewAngle(rightRearAngle);

        // Set the drive motor speeds
        leftRearUnit.setDriveSpeed(leftRearMagnitude);
        leftFrontUnit.setDriveSpeed(leftFrontMagnitude);
        rightFrontUnit.setDriveSpeed(rightFrontMagnitude);
        rightRearUnit.setDriveSpeed(rightRearMagnitude);
    }

    public void disable() {
        leftRearAngle = 0;
        leftFrontAngle = 0;
        rightFrontAngle = 0;
        rightRearAngle = 0;
        leftRearUnit.disable();
        leftFrontUnit.disable();
        rightFrontUnit.disable();
        rightRearUnit.disable();
    }
    public void stopDriveMotors(){
        leftRearUnit.setDriveSpeed(0);
        leftFrontUnit.setDriveSpeed(0);
        rightFrontUnit.setDriveSpeed(0);
        rightRearUnit.setDriveSpeed(0);
    }

    public double getLeftFrontDriveSpeed(){
        return leftFrontUnit.getDriveSpeed();
    }

    public double getAverageDriveSpeed(){
        return (leftFrontUnit.getIntegratedEncoderVelocity() + rightFrontUnit.getIntegratedEncoderVelocity() + leftRearUnit.getIntegratedEncoderVelocity() + rightRearUnit.getIntegratedEncoderVelocity())/4;
    }

    public void zero() {
        if (PINWHEEL_ZERO_ORIENTATION) {
            leftRearUnit.zeroModule(180);
            leftFrontUnit.zeroModule(-90);
            rightFrontUnit.zeroModule();
            rightRearUnit.zeroModule(90);
        } else {
            leftRearUnit.zeroModule();
            leftFrontUnit.zeroModule();
            rightFrontUnit.zeroModule();
            rightRearUnit.zeroModule();
        }
        final double leftRearSlewOffset = leftRearUnit.getSlewOffset();
        final double leftFrontSlewOffset = leftFrontUnit.getSlewOffset();
        final double rightFrontSlewOffset = rightFrontUnit.getSlewOffset();
        final double rightRearSlewOffset = rightRearUnit.getSlewOffset();
        saveSlewCalibration(String.format("%.4f,%.4f,%.4f,%.4f;\n", leftRearSlewOffset, leftFrontSlewOffset, rightFrontSlewOffset, rightRearSlewOffset));
    }

    /**
     * Saves the current data collected by the auton recorder to the specified file
     * path and clears the AutonRecorder.
     * 
     * @param autonFileName
     *            The path to the file to save the auton data to.
     */
    public synchronized void saveSlewCalibration(final String slewOffsets) {
        try {
            File calibrationFile = new File(String.format("/home/lvuser/%s.txt", "slewcalibration"));
            if (calibrationFile.exists()) {
                calibrationFile.delete();
            }
            calibrationFile.createNewFile();
            BufferedWriter writer = new BufferedWriter(new FileWriter(calibrationFile));
            writer.write(slewOffsets);
            // Flush the data to the file
            writer.flush();
            writer.close();
            System.out.println("Calibration Successfully Saved!");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * 
     * @param autonFileName
     * @return
     */
    public synchronized void loadSlewCalibration() {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(String.format("/home/lvuser/%s.txt", "slewcalibration")));
            String dataString = reader.readLine();

            // Parse the string
            dataString = dataString.trim();
            if (dataString.endsWith(";")) {
                dataString = dataString.substring(0, dataString.length() - 1);
                final String[] data = dataString.split(",");
                if (data.length == 4) {
                    final double leftRearSlewOffset = Double.parseDouble(data[0]);
                    final double leftFrontSlewOffset = Double.parseDouble(data[1]);
                    final double rightFrontSlewOffset = Double.parseDouble(data[2]);
                    final double rightRearSlewOffset = Double.parseDouble(data[3]);

                    leftRearUnit.setSlewOffset(leftRearSlewOffset);
                    leftFrontUnit.setSlewOffset(leftFrontSlewOffset);
                    rightFrontUnit.setSlewOffset(rightFrontSlewOffset);
                    rightRearUnit.setSlewOffset(rightRearSlewOffset);
                }
            }
            reader.close();
            System.out.println("Calibration Successfully Loaded!");
        } catch (IOException e) {
            System.err.printf("The Calibration File %s.txt could not be found!\n", "slewcalibration");
        }
    }

    public void SmartDashboard() {
        // Update Shuffleboard
        SmartDashboard.putString("Left Front Speed", String.format("%.2f", leftFrontUnit.getDriveSpeed()));
        SmartDashboard.putString("Left Rear Speed", String.format("%.2f", leftRearUnit.getDriveSpeed()));
        SmartDashboard.putString("Right Front Speed", String.format("%.2f", rightFrontUnit.getDriveSpeed()));
        SmartDashboard.putString("Right Rear Speed", String.format("%.2f", rightRearUnit.getDriveSpeed()));

        SmartDashboard.putString("Left Front Position", String.format("%.2f", leftFrontUnit.getIntegratedEncoderValue()));
        SmartDashboard.putString("Left Rear Position", String.format("%.2f", leftRearUnit.getIntegratedEncoderValue()));
        SmartDashboard.putString("Right Front Position", String.format("%.2f", rightFrontUnit.getIntegratedEncoderValue()));
        SmartDashboard.putString("Right Rear Position", String.format("%.2f", rightRearUnit.getIntegratedEncoderValue()));

        SmartDashboard.putString("Left Rear Slew Angle", String.format("%.2f | %.2f", leftRearUnit.getSlewAngle(), leftRearUnit.getSlewTargetAngle()));
        SmartDashboard.putString("Left Front Slew Angle", String.format("%.2f | %.2f", leftFrontUnit.getSlewAngle(), leftFrontUnit.getSlewTargetAngle()));
        SmartDashboard.putString("Right Front Slew Angle", String.format("%.2f | %.2f", rightFrontUnit.getSlewAngle(), rightFrontUnit.getSlewTargetAngle()));
        SmartDashboard.putString("Right Rear Slew Angle", String.format("%.2f | %.2f", rightRearUnit.getSlewAngle(), rightRearUnit.getSlewTargetAngle()));
    }

    public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
        return swerveDrivePoseEstimator;
    }

    public void updatePose(){
        //Update with encoders
        swerveDrivePoseEstimator.update(Robot.gyro.getRotation2d(), new SwerveModulePosition[]{
                leftFrontUnit.getSwerveModulePosition(),
                rightFrontUnit.getSwerveModulePosition(),
                leftRearUnit.getSwerveModulePosition(),
                rightRearUnit.getSwerveModulePosition()
        });

        /*
        //update with vision
        swerveDrivePoseEstimator.addVisionMeasurement(
                new Pose2d(Robot.limelight.getBOTPOSE()[0], Robot.limelight.getBOTPOSE()[1], new Rotation2d(Robot.limelight.getBOTPOSE()[4])),
                Robot.limelight.getBOTPOSE()[6]
                );

         */

    }

    public Pose2d getLLPose(){
        return new Pose2d(
                Robot.limelight.getBOTPOSE_WPIRED()[0],
                Robot.limelight.getBOTPOSE_WPIRED()[1],
                new Rotation2d(Robot.limelight.getBOTPOSE_WPIRED()[5])
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerveDriveKinematics.toChassisSpeeds(
                leftFrontUnit.getState(),
                rightFrontUnit.getState(),
                leftRearUnit.getState(),
                rightRearUnit.getState());
    }

    public SwerveModuleState[] getModuleState(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = leftFrontUnit.getState();
        states[1] = rightFrontUnit.getState();
        states[2] = leftRearUnit.getState();
        states[3] = rightRearUnit.getState();
        return states;
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(getRobotRelativeSpeeds(), 0.02);

        SwerveModuleState[] targetStates = swerveDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 0.35);
        leftFrontUnit.setTargetState(targetStates[0]);
        rightFrontUnit.setTargetState(targetStates[1]);
        leftRearUnit.setTargetState(targetStates[2]);
        rightRearUnit.setTargetState(targetStates[3]);
    }


    public Command followPathCommand(Pose2d targetPose){
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 1, Rotation2d.fromDegrees(0)),
                new Pose2d(5, 3, Rotation2d.fromDegrees(90)),
                targetPose
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI),
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
        );

        return new FollowPathHolonomic(
                path,
                this::getLLPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }// Reference to this subsystem to set requirements
        );
    }
}
