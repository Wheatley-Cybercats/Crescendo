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

package frc.robot.Subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 1;

  private final CANSparkFlex IntakeBot = new CANSparkFlex(5, MotorType.kBrushless);
  private final CANSparkFlex IntakeTop = new CANSparkFlex(4, MotorType.kBrushless);

  public IntakeIOSparkMax() {
    IntakeTop.restoreFactoryDefaults();
    IntakeBot.restoreFactoryDefaults();
    IntakeTop.setCANTimeout(250);
    IntakeBot.setCANTimeout(250);

    IntakeTop.setInverted(false);
    IntakeBot.follow(IntakeTop, true);

    IntakeTop.enableVoltageCompensation(12.0);
    IntakeTop.setSmartCurrentLimit(60);

    IntakeTop.burnFlash();
    IntakeBot.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    // inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() /
    // GEAR_RATIO
    // inputs.appliedVolts = IntakeTop.getAppliedOutput() * IntakeTop.getBusVoltage();
    inputs.currentAmps = new double[] {IntakeTop.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    IntakeTop.setVoltage(volts);
  }

  /*
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

   */

  @Override
  public void setSpeed(double speed) {
    IntakeTop.set(speed);
  }

  @Override
  public void stop() {
    IntakeTop.stopMotor();
  }
}
