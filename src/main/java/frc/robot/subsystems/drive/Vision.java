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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

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

  public double getNoteTX() {
    return io.getNoteTX();
  }

  public double getNoteTY() {
    return io.getNoteTY();
  }

  public boolean hasNote() {
    return io.hasNote();
  }

  public double getTagArea() {
    return io.getTagArea();
  }

  public Pose2d getVisionPose() {
    return io.getVisionPose();
  }

  public double getVisionTimestampPhoton() {
    return io.getVisionTimestampPhoton();
  }
}
