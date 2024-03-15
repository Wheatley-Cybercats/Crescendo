// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.*;
import frc.robot.Subsystems.climbers.Climber;
import frc.robot.Subsystems.climbers.ClimberIOSparkMax;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSparkMax;
import frc.robot.Subsystems.drive.Vision;
import frc.robot.Subsystems.drive.VisionIOLimelight;
import frc.robot.Subsystems.flywheel.Flywheel;
import frc.robot.Subsystems.flywheel.FlywheelIO;
import frc.robot.Subsystems.flywheel.FlywheelIOSim;
import frc.robot.Subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.indexer.IndexerIOSparkMax;
import frc.robot.Subsystems.intake.Intake;
import frc.robot.Subsystems.intake.IntakeIOSparkMax;
import frc.robot.Subsystems.leadscrew.Leadscrew;
import frc.robot.Subsystems.leadscrew.LeadscrewIO;
import frc.robot.Subsystems.leadscrew.LeadscrewIOSim;
import frc.robot.Subsystems.leadscrew.LeadscrewIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Leadscrew leadscrew;
  private final Indexer indexer;
  private final Intake intake;
  private final Vision vision;
  private final Climber climber;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new Vision(new VisionIOLimelight()));

        flywheel = new Flywheel(new FlywheelIOSparkMax());
        leadscrew = new Leadscrew(new LeadscrewIOTalonFX());
        indexer = new Indexer(new IndexerIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOLimelight());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new Vision(new VisionIOLimelight()));
        flywheel = new Flywheel(new FlywheelIOSim());
        leadscrew = new Leadscrew(new LeadscrewIOSim());
        indexer = new Indexer(new IndexerIOSparkMax()); // HMMHMMMHMHMHM SUS
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOLimelight());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new Vision(new VisionIOLimelight()));
        flywheel = new Flywheel(new FlywheelIO() {});
        leadscrew = new Leadscrew(new LeadscrewIO() {});
        indexer = new Indexer(new IndexerIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOLimelight());
        climber = new Climber(new ClimberIOSparkMax());
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    operatorController
        .b()
        .whileTrue(new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheel.SPEAKER));

    operatorController
        .a()
        .whileTrue(new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheel.AMP));

    operatorController.povUp().whileTrue(new MoveLeadScrewCommand(leadscrew, 0.17));

    operatorController.povDown().whileTrue(new MoveLeadScrewCommand(leadscrew, -0.17));

    operatorController
        .back() // Screen share button (left mid)
        .onTrue(new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrew.AMP));

    operatorController.leftBumper().whileTrue(new IntakeFromGroundCommand(intake, indexer));

    climber.setDefaultCommand(
        MoveClimberCommand.moveClimber(
            climber, operatorController::getLeftY, operatorController::getRightY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
