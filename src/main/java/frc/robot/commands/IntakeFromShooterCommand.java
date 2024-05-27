package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.led.Blinkin;

public class IntakeFromShooterCommand extends Command {
  private final Flywheel flywheel;
  private final Indexer indexer;
  private final Blinkin led;
  private long timer = 0;

  public IntakeFromShooterCommand(Flywheel flywheel, Indexer indexer, Blinkin led) {
    this.flywheel = flywheel;
    this.indexer = indexer;
    this.led = led;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.flywheel, this.indexer);
  }

  @Override
  public void initialize() {
    timer = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    flywheel.simpleVoltTop(0.3);
    indexer.setSpeed(0.15 * 1.5);
    led.strobe_gold();
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - timer > 2000 && !indexer.hasNote());
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    indexer.stop();
    led.set(0);
  }
}
