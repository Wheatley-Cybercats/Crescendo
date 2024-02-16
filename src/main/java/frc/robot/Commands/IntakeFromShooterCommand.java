package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotProperties;
import frc.robot.Subsystems.Flywheel;
import frc.robot.Subsystems.Indexer;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeFromShooterCommand extends Command {
    private final Flywheel flywheel = Robot.flywheel;
    private final Indexer indexer = Robot.indexer;

    private final DigitalInput beamBreak = new DigitalInput(0);

    public IntakeFromShooterCommand(Flywheel flywheel, Indexer indexer) {
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
        flywheel.setTopFlywheelMotorVolt(.3);
        indexer.setSpeed(RobotProperties.IndexerProperties.indexerIntakingSpeed*1.2);
        /*if (beamBreak.get()) {
            //set LED color
        } else{
            set current color
         */
        Robot.blinkin.rainbow_rainbow();
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
        Robot.blinkin.set(0);
        flywheel.resetPID();
    }
}
