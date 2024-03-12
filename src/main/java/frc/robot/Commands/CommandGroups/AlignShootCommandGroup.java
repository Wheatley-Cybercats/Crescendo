package frc.robot.Commands.CommandGroups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AlignHorizontallyCommand;
import frc.robot.Commands.IntakeFromGroundCommand;
import frc.robot.Commands.ShootSpeakerCommand;

public class AlignShootCommandGroup extends SequentialCommandGroup {
    public AlignShootCommandGroup() {
        super(new AlignHorizontallyCommand(), new ShootSpeakerCommand());
    }
}