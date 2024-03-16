package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.intake.Intake;
import frc.robot.Subsystems.led.Blinkin;

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
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
    blinkin.set(0);
  }
}
