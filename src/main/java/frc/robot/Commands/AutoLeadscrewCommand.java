// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.leadscrew.Leadscrew;
import org.littletonrobotics.junction.Logger;

public class AutoLeadscrewCommand extends Command {
  // private static final double LEADSCREW_CF = 300130; // do not set to zero will return NaN
  private final Leadscrew leadscrew;
  private final Translation3d target;
  private Drive drive;
  // private double leadscrewLegLength = Units.inchesToMeters(10.25);
  // private double leadscrewLegLength2 = Units.inchesToMeters(10.25);
  private double position = 0.0;

  /** Creates a new AutoLeadscrewCommand. */
  public AutoLeadscrewCommand(Leadscrew leadscrew, Translation3d target, Drive drive) {
    this.leadscrew = leadscrew;
    this.target = target;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leadscrew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /*
   * public static final double AMP_ANGLE = 91.5;
  public static final double PODIUM_ANGLE = 36; distance of 2.8 meters from wall 1 meter is 15 lead screw positions
  public static final double WING_ANGLE =
      21; // 17; //when front of bumpers are lined up with outer edge of stage distance of 3.9 meters from wall
  public static final double SUBWOOFER_ANGLE = 115;
   */
  @Override
  public void execute() {
    double hDelta = target.getZ() - leadscrew.getHeight();
    double angle = Math.atan(hDelta / target.getX() - drive.getPose().getX());
    if (angle < 0) {
      angle = 0;
    }
    if (drive.getPose().getX() < 1.75) {
      position = 115;
    } else if (drive.getPose().getX() > 1.75) {
      position = (-13.63636364 * (drive.getPose().getX()-1.75)) + 36;
    }

    /*
    position =
        Math.pow(leadscrewLegLength, 2)
            + Math.pow(leadscrewLegLength2, 2)
            - (2 * leadscrewLegLength * leadscrewLegLength2 * Math.cos(angle));
    position = Math.sqrt(position * LEADSCREW_CF) + 16;*/

    Logger.recordOutput("Lead Screw Anlge/Leadscrew", Math.toDegrees(angle));
    Logger.recordOutput("Lead Screw position/Leadscrew", position);
    leadscrew.runSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leadscrew.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leadscrew.atPosition(position);
  }
}
