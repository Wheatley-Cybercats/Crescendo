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

package frc.robot.Subsystems.leadscrew;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class LeadscrewIOTalonFX implements LeadscrewIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX motor = new TalonFX(10, "The CANivore");

  private final StatusSignal<Double> motorPosition = motor.getPosition();
  private final StatusSignal<Double> motorVelocity = motor.getVelocity();
  private final StatusSignal<Double> motorAppliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Double> motorCurrent = motor.getSupplyCurrent();
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  public LeadscrewIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(LeadscrewIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.positionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(motorVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {motorCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void runSetpoint(double setPointEncoderTicks) {
    if (motorPosition.getValueAsDouble()
        > setPointEncoderTicks) { // less negative: current position is higher than desired position
      moveShooter(
          Math.sqrt(Math.abs(motorPosition.getValueAsDouble() - setPointEncoderTicks)) / 1.5);
    } else if (motorPosition.getValueAsDouble()
        < setPointEncoderTicks) { // current position is lower than desired
      moveShooter(
          -Math.sqrt(Math.abs(motorPosition.getValueAsDouble() - setPointEncoderTicks)) / 4.5);
    } else if (atPosition(setPointEncoderTicks)) {
      stop();
    } else {
      stop();
    }
  }

  @Override
  public boolean atPosition(double position) {
    // the motor should stop whenever it is at a specific position
    return motorPosition.getValueAsDouble() - position < 1.2
        && motorPosition.getValueAsDouble() - position > -1.2;
  }

  @Override
  public void moveShooter(double speed) {
    motor.set(-speed);
    // thruBoreEncoder.getAbsolutePosition();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motor.getConfigurator().apply(config);
  }
}
