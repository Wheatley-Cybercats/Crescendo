package frc.robot.Commands.ShooterPresets;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.LeadScrew;


public class AmpAnglePreset extends Command {
    private final LeadScrew leadScrew;
    final double ampAnglePosition = 1000;

    public AmpAnglePreset(LeadScrew leadScrew) {
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
        leadScrew.moveToPosition(RobotProperties.AMP_ANGLE);
    }

    @Override
    public boolean isFinished() {
        if (leadScrew.atPosition(RobotProperties.AMP_ANGLE)){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        leadScrew.stop();
    }
}
