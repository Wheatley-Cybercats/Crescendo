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
import edu.wpi.first.math.util.Units;
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
import frc.robot.Subsystems.led.Blinkin;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handblinkin in the {@link
 * Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
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
  private final Blinkin blinkin;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        blinkin = new Blinkin();
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
        blinkin = new Blinkin();
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
        blinkin = new Blinkin();
        break;
    }

    NamedCommands.registerCommand(
        "shoot",
        new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.SPEAKER)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "intake", new IntakeFromGroundCommand(intake, indexer, blinkin).withTimeout(2));
    NamedCommands.registerCommand(
        "bumperup",
        new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.SUBWOOFER)
            .withTimeout(2));
    NamedCommands.registerCommand(
        "podium",
        new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.PODIUM)
            .withTimeout(2));
    NamedCommands.registerCommand(
        "wing",
        new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.WING).withTimeout(2));
    NamedCommands.registerCommand("IC", new IntakeFromGroundCommand(intake, indexer, blinkin));
    NamedCommands.registerCommand("OC", new OuttakeCommand(intake, indexer, blinkin));
    NamedCommands.registerCommand(
        "LSW", new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.WING));
    NamedCommands.registerCommand(
        "LSP", new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.AUTO));
    NamedCommands.registerCommand(
        "LSSW", new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.SUBWOOFER));
    NamedCommands.registerCommand(
        "FWS", new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.SPEAKER));
    NamedCommands.registerCommand(
        "AA",
        new AutoAllignCommand(
            drive,
            () -> 0.0,
            () -> 0.0,
            FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));
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
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .leftBumper() // .button(3) in sim
        .onTrue(
            Commands.runOnce(() -> drive.setMaxLinearSpeedMetersPerSec(Units.feetToMeters(7.5))));
    driverController
        .leftBumper() // .button(3)
        .onFalse(
            Commands.runOnce(() -> drive.setMaxLinearSpeedMetersPerSec(Units.feetToMeters(15))));
    driverController
        .b() // reset odometry pose
        .onTrue(Commands.runOnce(() -> drive.setYaw(0), drive).ignoringDisable(true));

    /** OPERATOR* */
    operatorController
        .b() // SHOOT SPEAKER
        .whileTrue(
            new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.SPEAKER));

    operatorController
        .a() // SHOOT AMP
        .whileTrue(new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.AMP));

    operatorController
        .povUp() // MOVE SHOOTER UP
        .whileTrue(new MoveLeadScrewCommand(leadscrew, 0.6));

    operatorController
        .povDown() // MOVE SHOOTER DOWN
        .whileTrue(new MoveLeadScrewCommand(leadscrew, -0.6));

    operatorController
        .start() // AMP ANGLE PRESET
        .onTrue(new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.AMP));

    operatorController
        .povRight() // PODIUM ANGLE PRESET
        .onTrue(new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.PODIUM));

    operatorController
        .povLeft() // WING ANGLE PRESET
        .onTrue(new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.WING));

    operatorController
        .y() // SUBWOOFER ANGLE PRESET
        .onTrue(new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.SUBWOOFER));

    operatorController
        .leftBumper()
        .whileTrue(new IntakeFromGroundCommand(intake, indexer, blinkin));

    operatorController.rightBumper().whileTrue(new OuttakeCommand(intake, indexer, blinkin));

    operatorController.x().whileTrue(new IntakeFromShooterCommand(flywheel, indexer, blinkin));

    climber.setDefaultCommand(
        MoveClimberCommand.moveClimber(
            climber, operatorController::getLeftY, operatorController::getRightY));

    operatorController
        .back()
        .and(operatorController.x())
        .onTrue(Commands.runOnce(() -> leadscrew.setPosition(115), leadscrew));
    driverController
        .button(1) // .button(1) for sim .y() for real
        .whileTrue(
            new AutoAllignCommand(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                FieldConstants.Speaker.centerSpeakerOpening
                    .toTranslation2d())); // FieldConstants.ampCenter returns negative infinity when
    // passed through autoalign
    driverController
        .button(2) // .button(2) for sim .a() for real
        .whileTrue(
            new AutoAllignCommand(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                FieldConstants.ampCenter)); //
    //
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
