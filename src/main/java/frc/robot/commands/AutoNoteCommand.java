package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Vision;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leadscrew.Leadscrew;
import frc.robot.subsystems.led.Blinkin;

public class AutoNoteCommand extends Command {
  private final Blinkin blinkin;
  private final Drive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Leadscrew leadscrew;
  private final Vision vision;

  public AutoNoteCommand(
      Blinkin blinkin,
      Drive drive,
      Indexer indexer,
      Intake intake,
      Leadscrew leadscrew,
      Vision vision) {
    this.blinkin = blinkin;
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.leadscrew = leadscrew;
    this.vision = vision;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(
        this.blinkin, this.drive, this.indexer, this.intake, this.leadscrew, this.vision);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (vision.hasTarget() && !indexer.hasNote()) {
      /*
      if (MathUtil.applyDeadband(vision.getNoteTX(), 0.2) > 0) {
        drive.runVelocity(new ChassisSpeeds(0, 0, -0.2));
      } else if (MathUtil.applyDeadband(vision.getNoteTX(), 0.2) < 0) {
        drive.runVelocity(new ChassisSpeeds(0, 0, 0.2));
      } else if (MathUtil.applyDeadband(vision.getNoteTY(), 0.2) > -5) {
        drive.runVelocity(new ChassisSpeeds(0.5, 0, 0));
      }

       */
      double noteTX = MathUtil.applyDeadband(vision.getTX(), 0.2);
      double noteTY = MathUtil.applyDeadband(vision.getTY(), 0.2);

      double omega = (noteTX > 0) ? -Math.pow(noteTX, 1.0 / 4) / 3 : Math.pow(noteTX, 1.0 / 4) / 3;
      double vx = (noteTY > -5) ? Math.pow(noteTY, 1.0 / 4) / 4 : 0;
      ChassisSpeeds speed = new ChassisSpeeds(vx, 0, omega);

      drive.runVelocity(speed);

      intake.setSpeed(-0.6);
      indexer.setSpeed(-0.17);
    }
  }

  @Override
  public boolean isFinished() {
    return indexer.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
  }
}
