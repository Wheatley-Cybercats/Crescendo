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

package frc.robot.subsystems.flywheel;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 1;

  public final CANSparkFlex top = new CANSparkFlex(39, MotorType.kBrushless);
  public final CANSparkFlex bottom = new CANSparkFlex(40, MotorType.kBrushless);
  private final RelativeEncoder encoder = top.getEncoder();
  private final SparkPIDController pid = top.getPIDController();

  public FlywheelIOSparkMax() {
    top.restoreFactoryDefaults();
    bottom.restoreFactoryDefaults();

    top.setCANTimeout(250);
    bottom.setCANTimeout(250);

    top.setInverted(false);

    top.enableVoltageCompensation(12.0);
    top.setSmartCurrentLimit(60);
    bottom.setSmartCurrentLimit(60);

    top.setIdleMode(CANSparkBase.IdleMode.kBrake);
    bottom.setIdleMode(CANSparkBase.IdleMode.kBrake);

    top.burnFlash();
    bottom.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVoltsTop = top.getAppliedOutput() * top.getBusVoltage();
    inputs.appliedVoltsBot = bottom.getAppliedOutput() * bottom.getBusVoltage();
    inputs.currentAmps = new double[] {top.getOutputCurrent(), bottom.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    top.setVoltage(volts);
  }

  @Override
  public void simpleVoltTop(double volts) {
    top.setVoltage(volts);
  }

  @Override
  public void simpleVoltBot(double volts) {
    bottom.setVoltage(volts);
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
  public boolean atSpeedTop(double speed) {
    return top.getEncoder().getVelocity() > speed;
  }

  @Override
  public boolean atSpeedBot(double speed) {
    return bottom.getEncoder().getVelocity() < speed;
  }

  @Override
  public double getVoltageTop() {
    return top.getAppliedOutput() * top.getBusVoltage();
  }

  @Override
  public double getVelocity() {
    return Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
  }

  @Override
  public CANSparkFlex getTopSparkFlex() {
    return top;
  }

  @Override
  public void stop() {
    top.stopMotor();
    bottom.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
