package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Blinkin;

public class IntakeFromGroundCommand extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final Blinkin blinkin;

  public IntakeFromGroundCommand(Intake intake, Indexer indexer, Blinkin blinkin) {
    this.intake = intake;
    this.indexer = indexer;
    this.blinkin = blinkin;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intake);
  }

  @Override
  public void execute() {
    if (!indexer.hasNote()) {
      intake.setSpeed(-0.6);
      indexer.setSpeed(-0.17);
    }
    blinkin.wave_forest();
  }

  @Override
  public boolean isFinished() {
    return indexer.hasNote();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
    blinkin.set(0);
  }
}
