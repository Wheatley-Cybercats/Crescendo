package frc.robot.Commands.ShooterPresets;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.LeadScrew;


public class SubwooferAnglePreset extends Command {
    private final LeadScrew leadScrew;

    public SubwooferAnglePreset(LeadScrew leadScrew) {
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
        leadScrew.moveToPosition(RobotProperties.SUBWOOFER_ANGLE);

    }

    @Override
    public boolean isFinished() {
        if (leadScrew.atPosition(RobotProperties.SUBWOOFER_ANGLE)){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        leadScrew.stop();

    }
}
