// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.leadscrew.Leadscrew;

public class AutoLeadscrewCommand extends Command {
  private static final double DISTANCE_PER_ROTATION = 0;
  private static final double GEAR_RATIO = 0;
  private final Leadscrew leadscrew;
  private final Translation3d target;
  private Drive drive;
  private double leadscrewLegLength = 0.0;
  private double leadscrewLegLength2 = 0.0;
  private double position = 0.0;

  /** Creates a new AutoLeadscrewCommand. */
  public AutoLeadscrewCommand(Leadscrew leadscrew, Translation3d target) {
    this.leadscrew = leadscrew;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leadscrew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double hDelta = target.getZ() - leadscrew.getHeight();
    double angle =
        Math.atan(target.getZ() - leadscrew.getHeight() / target.getX() - drive.getPose().getX());
    position =
        Math.pow(leadscrewLegLength, 2)
            + Math.pow(leadscrewLegLength2, 2)
            - (2 * leadscrewLegLength * leadscrewLegLength2 * Math.cos(angle));
    position = Math.sqrt(position / GEAR_RATIO * DISTANCE_PER_ROTATION);
    leadscrew.runSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leadscrew.atPosition(position);
  }
}
