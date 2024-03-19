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

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  public static enum Mode {
    WPI_RED,
    WPI_BLUE,
    WPI_CENTER
  }

  private final VisionIO io;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateIOInputs(visionInputs);
  }

  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
    Logger.recordOutput("Vision/Pipeline", pipeline);
  }

  public void setCamMode(int mode) {
    io.setCamMode(mode);
    Logger.recordOutput("Vision/CamMode", mode);
  }

  public Pose2d getPose(Mode mode) {
    if (mode.equals(Mode.WPI_BLUE)) {
      return io.getBotPose_WPIBLUE();
    } else if (mode.equals(Mode.WPI_RED)) {
      return io.getBotPose_WPIRED();
    } else if (mode.equals(Mode.WPI_CENTER)) {
      return io.getBotPose();
    }
    return new Pose2d();
  }

  public double getTimeStamp() {
    return io.getTimeStamp();
  }

  public boolean hasTarget() {
    return io.getHasTarget();
  }

  public double getTagCount() {
    return io.getTagCount();
  }
}
