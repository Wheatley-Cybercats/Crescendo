package frc.robot.Commands.CommandGroups;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.DriveToPointCommand;
import frc.robot.Commands.ShootTrapCommand;

public class TrapShootCommandGroup extends SequentialCommandGroup {
    public TrapShootCommandGroup() {
        super(
                new DriveToPointCommand(
                        new Pose2d() //TODO: Trap coords
                ),
                new ShootTrapCommand()
        );
    }
}