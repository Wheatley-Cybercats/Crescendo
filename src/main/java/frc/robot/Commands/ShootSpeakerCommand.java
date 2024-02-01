package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.Flywheel;
import frc.robot.Subsystems.Indexer;


public class ShootSpeakerCommand extends Command {
    private final Flywheel flywheel = Robot.flywheel;
    private final Indexer indexer = Robot.indexer;

    public ShootSpeakerCommand(Flywheel flywheel, Indexer indexer) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.flywheel, this.indexer);
    }

    @Override
    public void initialize() {
        flywheel.resetPID();
    }

    @Override
    public void execute() {
        flywheel.setTopFlywheelMotorVolt(7);
        flywheel.setBotFlywheelMotorVolt(-7);
        if(flywheel.topflywheelAtSpeed() && flywheel.botflywheelAtSpeed()){
            indexer.setSpeed(RobotProperties.IndexerProperties.shootingSpeed);
        }
        else{
            indexer.stop();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        indexer.stop();
        flywheel.resetPID();
    }
}
