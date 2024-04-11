package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leadscrew.Leadscrew;

public class MoveLeadScrewCommand extends Command {
  private final Leadscrew leadscrew;
  private final double speed;

  public MoveLeadScrewCommand(Leadscrew leadscrew, double speed) {
    this.leadscrew = leadscrew;
    this.speed = speed;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.leadscrew);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    leadscrew.setSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    leadscrew.stop();
  }
}
