// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Subsystems.climbers;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 1;

  private final TalonFX ClimbRight = new TalonFX(20, "The CANivore");
  private final TalonFX ClimbLeft = new TalonFX(21, "The CANivore");

  public ClimberIOSparkMax() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    // inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() /
    // GEAR_RATIO);
    // inputs.appliedVolts = Climb.getAppliedOutput() * Climb.getBusVoltage();
    // inputs.currentAmps = new double[] {ClimbLeft.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    ClimbLeft.setVoltage(volts);
  }

  @Override
  public void setSpeedLeft(double speed) {
    ClimbLeft.set(speed);
  }

  @Override
  public void setSpeedRight(double speed) {
    ClimbRight.set(speed);
  }

  @Override
  public void stop(int leftRight) {
    if (leftRight == 0) {
      ClimbLeft.stopMotor();
    } else {
      ClimbRight.stopMotor();
    }
  }
}
