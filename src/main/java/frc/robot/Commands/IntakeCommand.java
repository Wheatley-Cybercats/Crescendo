package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;


public class IntakeCommand extends Command {
    private final Intake intake = Robot.intake;
    private final Indexer indexer = Robot.indexer;

    public IntakeCommand(Intake intake, Indexer indexer) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setSpeed(RobotProperties.IntakeProperties.intakeIntakingSpeed);
        indexer.setSpeed(-RobotProperties.IndexerProperties.indexerIntakingSpeed);
    }

    @Override
    public boolean isFinished() {

        //TODO: when beam break detects note, this will return true, which will call end()

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        intake.stop();
    }
}
