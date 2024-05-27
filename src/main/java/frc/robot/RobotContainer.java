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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.PresetFlywheelCommand;
import frc.robot.subsystems.climbers.Climber;
import frc.robot.subsystems.climbers.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.Vision;
import frc.robot.subsystems.drive.VisionIOPhotonLight;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.leadscrew.Leadscrew;
import frc.robot.subsystems.leadscrew.LeadscrewIO;
import frc.robot.subsystems.leadscrew.LeadscrewIOSim;
import frc.robot.subsystems.leadscrew.LeadscrewIOTalonFX;
import frc.robot.subsystems.led.Blinkin;
import frc.robot.util.NoteVisualizer;
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
  private Boolean autoMode = true;

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
                new Vision(new VisionIOPhotonLight()));

        flywheel = new Flywheel(new FlywheelIOSparkMax());
        leadscrew = new Leadscrew(new LeadscrewIOTalonFX());
        indexer = new Indexer(new IndexerIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOPhotonLight());
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
                new Vision(new VisionIOPhotonLight()));
        flywheel = new Flywheel(new FlywheelIOSim());
        leadscrew = new Leadscrew(new LeadscrewIOSim());
        indexer = new Indexer(new IndexerIOSparkMax()); // HMMHMMMHMHMHM SUS
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOPhotonLight());
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
                new Vision(new VisionIOPhotonLight()));
        flywheel = new Flywheel(new FlywheelIO() {});
        leadscrew = new Leadscrew(new LeadscrewIO() {});
        indexer = new Indexer(new IndexerIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        vision = new Vision(new VisionIOPhotonLight());
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
        "LSPW", new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.SUBWOOFER));
    NamedCommands.registerCommand(
        "FWS",
        new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.SPEAKER)
            .andThen(NoteVisualizer.shoot()));
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

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX() * .85));
    driverController
        .leftBumper() // .button(3) in sim
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * .5,
                () -> -driverController.getLeftX() * .5,
                () -> -driverController.getRightX() * .5));

    driverController
        .b() // reset odometry pose
        .onTrue(Commands.runOnce(() -> drive.setYaw(0), drive).ignoringDisable(true));
    Trigger rumbleTrigger = new Trigger(indexer::hasNote);
    rumbleTrigger.onTrue(
        Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5))
            .withTimeout(1)
            .andThen(
                Commands.runOnce(
                    () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))));
    /** OPERATOR* */
    operatorController
        .b() // SHOOT SPEAKER
        .whileTrue(
            new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.SPEAKER)
                .andThen(NoteVisualizer.shoot()));
    operatorController
        .a() // SHOOT AMP
        .whileTrue(
            new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.AMP)
                .andThen(NoteVisualizer.shoot()));
    operatorController
        .povUp() // MOVE SHOOTER UP
        .whileTrue(
            new MoveLeadScrewCommand(leadscrew, 0.6)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .povDown() // MOVE SHOOTER DOWN
        .whileTrue(
            new MoveLeadScrewCommand(leadscrew, -0.6)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .start() // AMP ANGLE PRESET
        .onTrue(
            new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.AMP)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .povRight() // PODIUM ANGLE PRESET
        .onTrue(
            new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.PODIUM)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .povLeft() // WING ANGLE PRESET
        .onTrue(
            new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.WING)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .y() // SUBWOOFER ANGLE PRESET
        .onTrue(
            new PresetLeadscrewCommand(leadscrew, Constants.PresetLeadscrewAngle.SUBWOOFER)
                .alongWith(Commands.runOnce(() -> autoMode = false)));

    operatorController
        .leftBumper()
        .whileTrue(new IntakeFromGroundCommand(intake, indexer, blinkin));

    operatorController.rightBumper().whileTrue(new OuttakeCommand(intake, indexer, blinkin));

    operatorController.x().whileTrue(new IntakeFromShooterCommand(flywheel, indexer, blinkin));
    operatorController
        .back()
        .and(operatorController.povUp())
        .onTrue(new AutoLeadscrewCommand(leadscrew, FieldConstants.Speaker.centerSpeakerOpening));
    // .onTrue(Commands.runOnce(() -> autoMode = false));

    climber.setDefaultCommand(
        MoveClimberCommand.moveClimber(
            climber, operatorController::getLeftY, operatorController::getRightY));

    operatorController
        .back()
        .and(operatorController.x())
        .onTrue(Commands.runOnce(() -> leadscrew.setPosition(115), leadscrew));

    operatorController
        .back()
        .and(operatorController.a())
        .whileTrue(new PresetFlywheelCommand(indexer, flywheel, Constants.PresetFlywheelSpeed.LOB));

    driverController
        .x() // .button(1) for sim .y() for real
        .whileTrue(
            new AutoAllignCommand(
                drive,
                    driverController::getLeftX,
                    driverController::getLeftY,
                FieldConstants.Speaker.centerSpeakerOpening
                    .toTranslation2d())); // FieldConstants.ampCenter returns negative infinity when

    driverController
        .a() // .button(2) for sim .a() for real
        .whileTrue(
            new AutoNotePickupCommand(blinkin, drive, indexer, intake, leadscrew, vision)); //

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
