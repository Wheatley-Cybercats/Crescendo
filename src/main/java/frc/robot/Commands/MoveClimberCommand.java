package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.climbers.Climber;
import java.util.function.DoubleSupplier;

public class MoveClimberCommand {
  public static Command moveClimber(
      Climber climber, DoubleSupplier ySupplierLeft, DoubleSupplier ySupplierRight) {

    return Commands.run(
        () -> {
          climber.setSpeedLeft(MathUtil.applyDeadband(ySupplierLeft.getAsDouble(), 0.5));
          climber.setSpeedRight(MathUtil.applyDeadband(ySupplierRight.getAsDouble(), 0.5));
        },
        climber);
  }
}
