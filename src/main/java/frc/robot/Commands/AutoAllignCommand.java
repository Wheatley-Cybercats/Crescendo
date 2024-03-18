// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;

public class AutoAllignCommand extends Command {
  private final double xPosition;
  private final double yPosition;
  private final Drive drive;
  private double heading;

  /** Creates a new AutoAllignCommand. */
  public AutoAllignCommand(Drive drive, double xPosition, double yPosition, double heading) {
    this.drive = drive;
    this.xPosition = xPosition;
    this.yPosition = yPosition;
    this.heading = heading;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.pathFind(new Pose2d(xPosition, yPosition, heading));
    if (drive.getPose().getRotation().getDegrees()
        < heading) { // less negative: current position is higher than desired position
      drive.runVelocity(
          new ChassisSpeeds(
              0, 0, Math.sqrt(Math.abs(drive.getPose().getRotation().getDegrees() - heading))));
    } else if (drive.getPose().getRotation().getDegrees()
        > heading) { // current position is lower than desired
      drive.runVelocity(
          new ChassisSpeeds(
              0, 0, -(Math.sqrt(Math.abs(drive.getPose().getRotation().getDegrees() - heading)))));
    } else if (drive.getPose().getRotation().getDegrees() == heading) {
      drive.stop();
    } else {
      drive.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getPose().getRotation().getDegrees() == heading;
  }
}
