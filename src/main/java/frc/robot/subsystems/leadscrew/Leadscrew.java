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

package frc.robot.subsystems.leadscrew;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Leadscrew extends SubsystemBase {
  private static final double HEIGHT_CONVERSION =
      36
          / Units.inchesToMeters(
              16); // Measure physical height of leadscrew and map them with the encoder values than
  // need to reeasure for podium shot
  // find a
  // constant that best fits and plug in
  private final LeadscrewIO io;
  private final LeadscrewIOInputsAutoLogged inputs = new LeadscrewIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  /** Creates a new Leadscrew. */
  public Leadscrew(LeadscrewIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Leadscrew/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SmartDashboard.putNumber("Lead Screw Position", inputs.position);
    Logger.processInputs("Leadscrew", inputs);
  }

  public double getHeight() {
    return inputs.position * HEIGHT_CONVERSION;
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log Leadscrew setpoint
    Logger.recordOutput("Leadscrew/SetpointRPM", velocityRPM);
  }

  public void runSetpoint(double setEncoderRotations) {
    io.runSetpoint(setEncoderRotations);
    Logger.recordOutput("Leadscrew/SetpointRotations", setEncoderRotations);
  }

  public boolean atPosition(double position) {
    return io.atPosition(position);
  }

  public void moveShooter(double speed) {
    io.moveShooter(-speed);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return io.getPosition();
  }

  /** Stops the Leadscrew. */
  public void stop() {
    io.stop();
  }
}
