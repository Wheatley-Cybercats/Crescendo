package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.intake.Intake;


public class OuttakeCommand extends Command {
    private final Indexer indexer;
    private final Intake intake;

    public OuttakeCommand(Intake intake, Indexer indexer) {
        this.indexer = indexer;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexer, this.intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setSpeed(0.8);
        indexer.setSpeed(0.2);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }
}
