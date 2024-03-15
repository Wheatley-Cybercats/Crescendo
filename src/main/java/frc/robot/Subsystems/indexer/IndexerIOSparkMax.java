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

package frc.robot.Subsystems.indexer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IndexerIOSparkMax implements IndexerIO {
  private static final double GEAR_RATIO = 1;
  private final CANSparkFlex indexer = new CANSparkFlex(9, MotorType.kBrushless);
  private final RelativeEncoder encoder = indexer.getEncoder();
  private final SparkPIDController pid = indexer.getPIDController();
  private final DigitalInput dio = new DigitalInput(0);

  public IndexerIOSparkMax() {
    indexer.restoreFactoryDefaults();

    indexer.setCANTimeout(250);

    indexer.setInverted(false);

    indexer.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.currentAmps = new double[] {indexer.getOutputCurrent()};
    inputs.hasNote = dio.get();
  }

  @Override
  public void setVoltage(double volts) {
    indexer.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setSpeed(double speed) {
    indexer.set(speed);
  }

  @Override
  public boolean hasNote() {
    return !dio.get();
  }

  @Override
  public void stop() {
    indexer.stopMotor();
  }
}
