package frc.robot.Commands.ShooterPresets;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LeadScrew;


public class SourceAnglePreset extends Command {
    private final LeadScrew leadScrew;


    public SourceAnglePreset(LeadScrew leadScrew) {
        this.leadScrew = leadScrew;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.leadScrew);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

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
