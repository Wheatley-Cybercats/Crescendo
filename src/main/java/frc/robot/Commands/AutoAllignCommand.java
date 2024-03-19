// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class AutoAllignCommand extends Command {
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Drive drive;
  private double targetHeading;
  private double DEADBAND = 0.1;
  private Translation2d linearVelocity;
  private Translation2d target;

  /** Creates a new AutoAllignCommand. */
  public AutoAllignCommand(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d target) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.target = target;
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearMagnitude = linearMagnitude * linearMagnitude;
    linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    targetHeading =
        Math.toDegrees(
            Math.atan(
                (target.getY() - drive.getPose().getY())
                    / (target.getX() - drive.getPose().getX())));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    /*if(isFlipped){
      drive.getRotation().plus(new Rotation2d(Math.PI));
    }*/
    targetHeading =
        Math.toDegrees(
            Math.atan(
                (target.getY() - drive.getPose().getY())
                    / (target.getX() - drive.getPose().getX())));
    if (drive.getPose().getRotation().getDegrees() < targetHeading) {
      drive.runVelocity(
          new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              Math.sqrt(
                  Math.abs(drive.getPose().getRotation().getDegrees() - targetHeading) / 4.5))); //
    } else if (drive.getPose().getRotation().getDegrees() > targetHeading) {
      drive.runVelocity(
          new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              -(Math.sqrt(Math.abs(drive.getPose().getRotation().getDegrees() - targetHeading))
                  / 4.5)));
    } else if (drive.getPose().getRotation().getDegrees() == targetHeading) {
      drive.stop();
    } else {
      drive.stop();
    }
  }

  public boolean atPosition() {
    return drive.getPose().getRotation().getDegrees() - targetHeading < 1.2
        && drive.getPose().getRotation().getDegrees() - targetHeading > -1.2;
    // the motor should stop whenever it is at a specific position

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition();
  }
}
