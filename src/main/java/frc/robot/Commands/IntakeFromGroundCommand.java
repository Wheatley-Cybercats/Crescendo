package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Indexer;
import frc.robot.Intake;

public class IntakeFromGroundCommand extends Command {
    private final Intake intake = Robot.intake;
    private final Indexer indexer = Robot.indexer;
    //private final DigitalInput beamBreak = Robot.beamBreak;

    public IntakeFromGroundCommand(Intake intake, Indexer indexer) {
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
        //when beam break detects note, this will return true & call end()
        //if (indexer.getBeamBreakState()) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        intake.stop();
    }
}
