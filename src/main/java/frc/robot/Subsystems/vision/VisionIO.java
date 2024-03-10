// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public double captureTimestamp = 0.0;
    public double[] cornerX = new double[] {};
    public double[] cornerY = new double[] {};
    public boolean simpleValid = false;
    public double simpleAngle = 0.0;

    public void toLog(LogTable table) {
      table.put("Capture Timestamp", captureTimestamp);
      table.put("Corner X", cornerX);
      table.put("Corner Y", cornerY);
      table.put("Simple Valid", simpleValid);
      table.put("Simple Angle", simpleAngle);
    }

    public void fromLog(LogTable table) {
      captureTimestamp = table.get("Capture Timestamp", captureTimestamp);
      cornerX = table.get("Corner X", cornerX);
      cornerY = table.get("Corner Y", cornerY);
      simpleValid = table.get("Simple Valid", simpleValid);
      simpleAngle = table.get("Simple Angle", simpleAngle);
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setLeds(boolean on) {}

  public default void setPipeline(int pipeline) {}
}
