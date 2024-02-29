package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;


public class OuttakeCommand extends Command {
    private final Intake intake = Robot.intake;
    private final Indexer indexer = Robot.indexer;

    public OuttakeCommand(Intake intake, Indexer indexer) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setSpeed(-RobotProperties.IntakeProperties.intakeIntakingSpeed);
        indexer.setSpeed(RobotProperties.IndexerProperties.indexerIntakingSpeed);
        Robot.blinkin.strobe_gold();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        intake.stop();
    }
}
