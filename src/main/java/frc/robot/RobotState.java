// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.swerve.ModuleLimits;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class RobotState {
  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
    public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
      double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
      double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
      return new FlywheelSpeeds(leftSpeed, rightSpeed);
    }
  }

  public record AimingParameters(
      Rotation2d driveHeading,
      Rotation2d armAngle,
      double effectiveDistance,
      FlywheelSpeeds flywheelSpeeds) {}

  private static final LoggedTunableNumber autoLookahead =
      new LoggedTunableNumber("RobotState/AutoLookahead", 0.5);
  private static final LoggedTunableNumber lookahead =
      new LoggedTunableNumber("RobotState/lookaheadS", 0.35);
  private static final LoggedTunableNumber closeShootingZoneFeet =
      new LoggedTunableNumber("RobotState/CloseShootingZoneFeet", 10.0);
  private static final double poseBufferSizeSeconds = 2.0;

  @AutoLogOutput @Getter @Setter private boolean flywheelAccelerating = false;
  @AutoLogOutput @Getter @Setter private double shotCompensationDegrees = 0.0;

  private static final double autoFarShotCompensationDegrees = 0.0; // 0.6 at NECMP

  public void adjustShotCompensationDegrees(double deltaDegrees) {
    shotCompensationDegrees += deltaDegrees;
  }

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  @Getter @Setter private Pose2d trajectorySetpoint = new Pose2d();
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
  private Twist2d trajectoryVelocity = new Twist2d();

  /** Cached latest aiming parameters. Calculated in {@code getAimingParameters()} */
  private AimingParameters latestParameters = null;

  @Setter private BooleanSupplier lookaheadDisable = () -> false;

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = DriveConstants.kinematics;

    // Setup NoteVisualizer
    NoteVisualizer.setRobotPoseSupplier(this::getEstimatedPose);
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    latestParameters = null;
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    // Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }
    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    latestParameters = null;
    this.robotVelocity = robotVelocity;
  }

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    latestParameters = null;
    trajectoryVelocity = robotVelocity;
  }

  public AimingParameters getAimingParameters() {
    if (latestParameters != null) {
      // Cache previously calculated aiming parameters. Cache is invalidated whenever new
      // observations are added.
      return latestParameters;
    }

    Transform2d fieldToTarget =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
            .toTranslation2d()
            .toTransform2d()
            .plus(FudgeFactors.speaker.getTransform());
    Pose2d fieldToPredictedVehicle;
    if (DriverStation.isAutonomousEnabled()) {
      fieldToPredictedVehicle = getPredictedPose(autoLookahead.get(), autoLookahead.get());

    } else {
      fieldToPredictedVehicle =
          lookaheadDisable.getAsBoolean()
              ? getEstimatedPose()
              : getPredictedPose(lookahead.get(), lookahead.get());
    }
    Logger.recordOutput("RobotState/AimingParameters/PredictedPose", fieldToPredictedVehicle);

    Pose2d fieldToPredictedVehicleFixed =
        new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

    Translation2d predictedVehicleToTargetTranslation =
        fieldToPredictedVehicle.inverse().transformBy(fieldToTarget).getTranslation();
    Translation2d predictedVehicleFixedToTargetTranslation =
        fieldToPredictedVehicleFixed.inverse().transformBy(fieldToTarget).getTranslation();

    Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
    double targetDistance = predictedVehicleToTargetTranslation.getNorm();

    double autoFarArmCorrection =
        DriverStation.isAutonomousEnabled() && targetDistance >= Units.inchesToMeters(125)
            ? autoFarShotCompensationDegrees
            : 0.0;
    Logger.recordOutput(
        "RobotState/AimingParameters/AutoFarArmCorrectionDegrees", autoFarArmCorrection);
    latestParameters =
        new AimingParameters(
            targetVehicleDirection,
            Rotation2d.fromDegrees(
                +shotCompensationDegrees + autoFarArmCorrection), // TODO: armDegreeAngle?
            targetDistance,
            new FlywheelSpeeds(0, 0));
    return latestParameters;
  }

  public ModuleLimits getModuleLimits() {
    return flywheelAccelerating && !DriverStation.isAutonomousEnabled()
        ? DriveConstants.moduleLimitsFlywheelSpinup
        : DriveConstants.moduleLimitsFree;
  }

  public boolean inShootingZone() {
    Pose2d robot = AllianceFlipUtil.apply(getEstimatedPose());
    if (robot.getY() <= FieldConstants.Stage.ampLeg.getY()) {
      return robot.getX() <= FieldConstants.wingX;
    } else {
      return robot.getX() <= FieldConstants.fieldLength / 2.0 + 0.5;
    }
  }

  public boolean inCloseShootingZone() {
    return getEstimatedPose()
            .getTranslation()
            .getDistance(
                AllianceFlipUtil.apply(
                    FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
        < Units.feetToMeters(closeShootingZoneFeet.get());
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    odometryPose = initialPose;
    poseBuffer.clear();
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}