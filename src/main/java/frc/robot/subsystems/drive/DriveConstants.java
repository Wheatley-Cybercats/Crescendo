// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Builder;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  // Odometry Constants

  public static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
  public static final DriveConfig driveConfig =
      DriveConfig.builder()
          .wheelRadius(Units.inchesToMeters(1.942)) // 1.936
          .trackWidthX(Units.inchesToMeters(20.75))
          .trackWidthY(Units.inchesToMeters(20.75))
          .bumperWidthX(Units.inchesToMeters(37))
          .bumperWidthY(Units.inchesToMeters(33))
          .maxLinearVelocity(Units.feetToMeters(15.0))
          .maxLinearAcceleration(Units.feetToMeters(75.0))
          .maxAngularVelocity(12.0)
          .maxAngularAcceleration(6.0)
          .build();
  public static final TrajectoryConstants trajectoryConstants =
      new TrajectoryConstants(
          6.0,
          0.0,
          8.0,
          0.0,
          Units.inchesToMeters(4.0),
          Units.degreesToRadians(5.0),
          Units.inchesToMeters(5.0),
          Units.degreesToRadians(7.0),
          driveConfig.maxLinearVelocity() / 2.0,
          driveConfig.maxAngularVelocity() / 2.0);
  // Module Constants
  public static final ModuleConstants moduleConstants =
      new ModuleConstants(
          0.064773,
          0.10267,
          1.0 / DCMotor.getNeoVortex(1).KtNMPerAmp,
          0,
          0,
          0,
          0,
          Mk4iReductions.L3.reduction,
          Mk4iReductions.TURN.reduction);
  public static final HeadingControllerConstants headingControllerConstants =
      new HeadingControllerConstants(5.0, 0.0, 8.0, 20.0);
  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          driveConfig.maxLinearVelocity(),
          driveConfig.maxLinearAcceleration(),
          Units.degreesToRadians(1080.0));

  @Builder
  public record DriveConfig(
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }
  }

  @Builder
  public record TrajectoryConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double goalLinearTolerance,
      double goalThetaTolerance,
      double linearVelocityTolerance,
      double angularVelocityTolerance) {}

  public record ModuleConstants(
      double ffkS,
      double ffkV,
      double ffkT,
      double drivekP,
      double drivekD,
      double turnkP,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record AutoAlignConstants(
      double linearkP,
      double linearkD,
      double thetakP,
      double thetakD,
      double linearTolerance,
      double thetaTolerance,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {}

  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
