package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.lib.util.LimeLightWrapper;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PigeonYummy;

import java.util.Map;

public class SwerveBase extends SubsystemBase {

    public LimeLightWrapper cam = new LimeLightWrapper();

    public SwerveDrivePoseEstimator swerveOdometer;
    public RevSwerveModule[] swerveMods;
    public PigeonYummy gyro = PigeonYummy.getInstance();

    private int moduleSynchronizationCounter = 0;
    private double avgOmega = 0;

//    private Rotation2d fieldOffset = new Rotation2d(gyro.getYaw()).rotateBy(new Rotation2d(180));
    private final Field2d field = new Field2d();
    private boolean hasInitialized = false;

    private GenericEntry aprilTagTarget = RobotContainer.autoTab
            .add("Currently Seeing April Tag", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "green", "Color when false", "red"))
            .withPosition(8, 4).withSize(2, 2).getEntry();



    public SwerveBase() {

        swerveMods = new RevSwerveModule[] {

            new RevSwerveModule(0, Constants.Swerve.Modules.Mod0.constants),
            new RevSwerveModule(1, Constants.Swerve.Modules.Mod1.constants),
            new RevSwerveModule(2, Constants.Swerve.Modules.Mod2.constants),
            new RevSwerveModule(3, Constants.Swerve.Modules.Mod3.constants)
        };

        swerveOdometer = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        zeroGyro();

    }

    public void wheelsIn() {
        //TODO: I think this should be used somewhere...
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
                false);
    }

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        //TODO: maybe try calculations without correct for dynamics because I'm thinking dynamics could be very different for everyone based on their robot
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveOdometer.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(DriverStation.getAlliance() == DriverStation.Alliance.Red ? 180 : 0))
        )
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
        );
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        //TODO: THIS IS THE MOST LIKELY PLACE TO BE WRONG
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }
    public Pose2d getPose() {
        return swerveOdometer.getEstimatedPosition();
    }
    public void resetOdometry(Pose2d pose) {

        swerveOdometer.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());

    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(double deg) {
        gyro.resetGyro(); //deg parameter isn't being used ...

    }

    public void zeroGyro() {
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360).minus(gyro.getRotation2d()) : gyro.getRotation2d();
    }
    public double getPitch() {
        return gyro.getPitch(); //TODO: could be roll or pitch figure it out!
    }

    public void synchronizeModuleEncoders() {
        for(RevSwerveModule mod : swerveMods) {
            mod.synchronizeEncoders();
        }
    }
    public double getAvgOmega() {
        double sum = 0;
        for(RevSwerveModule mod : swerveMods) {
            sum += Math.abs(mod.getOmega());
        }
        return sum / 4;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Mod 0 abs ", swerveMods[0].getAbsAngle());
        SmartDashboard.putNumber("Mod 1 abs ", swerveMods[1].getAbsAngle());
        SmartDashboard.putNumber("Mod 2 abs ", swerveMods[2].getAbsAngle());
        SmartDashboard.putNumber("Mod 3 abs ", swerveMods[3].getAbsAngle());

        //SmartDashboard.putNumber("NavxHEADING", gyro.getHeading());

        swerveOdometer.update(getYaw(), getModulePositions());
        SmartDashboard.putBoolean("photonGood", cam.latency() < 0.6);
        if (!hasInitialized /* || DriverStation.isDisabled() */) {
            var robotPose = cam.getPose();
                swerveOdometer.resetPosition(getYaw(), getModulePositions(), robotPose);
                hasInitialized = true;
        } else {
            var result = cam.getEstimatedGlobalPose(swerveOdometer);
                if (cam.seesTarget()) {
                    swerveOdometer.addVisionMeasurement(result.getEstimatedPosition(),
                            cam.latency());
                }
                field.getObject("Cam Est Pose").setPose(cam.getEstimatedGlobalPose(swerveOdometer).getEstimatedPosition());
        }

        SmartDashboard.putData("field", field);

        field.setRobotPose(getPose());
        aprilTagTarget.setBoolean(cam.seesTarget());

        avgOmega = getAvgOmega();

        for(SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
        // lib)
        // To ensure that everytime we initialize it works.
        if (avgOmega <= .03 && ++moduleSynchronizationCounter > 20)
        {
            SmartDashboard.putBoolean("Synchronizing Encoders", !SmartDashboard.getBoolean("Synchronizing Encoders", false));
            synchronizeModuleEncoders();
            moduleSynchronizationCounter = 0;
        }
        if(avgOmega <= .005){
            SmartDashboard.putBoolean("Can Synchronizing Encoders", true);
        }else {
            SmartDashboard.putBoolean("Can Synchronizing Encoders", false);
        }
        SmartDashboard.putNumber("avgOmega", avgOmega);

        SmartDashboard.putBoolean("isRed", DriverStation.getAlliance() == DriverStation.Alliance.Red);
    }

    public void stop() {
        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(mod.getState(), false);
        }
    }
}
