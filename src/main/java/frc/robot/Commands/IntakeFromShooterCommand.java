package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.flywheel.Flywheel;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.led.Blinkin;

public class IntakeFromShooterCommand extends Command {
    private final Flywheel flywheel;
    private final Indexer indexer;
    private final Blinkin led;

    //private final DigitalInput beamBreak = new DigitalInput(0);

    public IntakeFromShooterCommand(Flywheel flywheel, Indexer indexer, Blinkin led) {
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.led = led;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.flywheel, this.indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flywheel.simpleVoltTop(0.3);
        indexer.setSpeed(0.15*1.5);
        led.solid_green();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;//indexer.getBeamBreakState();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        indexer.stop();
        led.set(0);
    }
}