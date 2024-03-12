package frc.robot.Commands.ShooterPresets;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.LeadScrew;


public class AutoAnglePreset extends Command {
    private final LeadScrew leadScrew = Robot.leadscrew;

    public AutoAnglePreset() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        leadScrew.moveToPosition(45);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
