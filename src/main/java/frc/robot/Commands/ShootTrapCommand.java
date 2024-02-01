package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.Flywheel;
import frc.robot.Subsystems.Indexer;


public class ShootTrapCommand extends Command {
    private final Flywheel flywheel;
    private final Indexer indexer;

    public ShootTrapCommand(Flywheel flywheel, Indexer indexer) {
        this.flywheel = flywheel;
        this.indexer = indexer;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.flywheel, this.indexer);
    }

    @Override
    public void initialize() {
        flywheel.resetPID();
    }

    @Override
    public void execute() {
        flywheel.setTopFlywheelMotorVolt(4.5);
        flywheel.setBotFlywheelMotorVolt(-4.5);
        if(flywheel.topflywheelAtTrapSpeed() && flywheel.botflywheelAtTrapSpeed()){
            indexer.setSpeed(RobotProperties.IndexerProperties.shootingSpeed);
        }
        else{
            indexer.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        indexer.stop();
        flywheel.resetPID();
    }
}
