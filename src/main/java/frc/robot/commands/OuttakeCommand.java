package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Blinkin;

public class OuttakeCommand extends Command {
  private final Indexer indexer;
  private final Intake intake;
  private final Blinkin blinkin;

  public OuttakeCommand(Intake intake, Indexer indexer, Blinkin blinkin) {
    this.indexer = indexer;
    this.intake = intake;
    this.blinkin = blinkin;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexer, this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setSpeed(0.8);
    indexer.setSpeed(0.2);
    blinkin.solid_red();
  }

  @Override
  public boolean isFinished() {
    // Doesn't need this because it is whileTrue
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
    blinkin.set(0);
  }
}
