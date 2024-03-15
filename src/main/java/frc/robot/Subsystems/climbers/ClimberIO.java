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

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRadLeft = 0.0;
    public double positionRadRight = 0.0;
    public double velocityRadPerLeft = 0.0;
    public double velocityRadPerRight = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double appliedVoltsRight = 0.0;
    public double[] currentAmpsLeft = new double[] {};
    public double[] currentAmpsRight = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  // public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void setSpeed(double speed) {}

  public default void setSpeedLeft(double speed) {}
  ;

  public default void setSpeedRight(double speed) {}
  ;

  /** Stop in open loop. */
  public default void stop() {}
}
