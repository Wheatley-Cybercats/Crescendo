package frc.robot;// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Team 3171 Imports
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import frc.team2872.drive.SwerveDrive;
import frc.team2872.sensors.Pigeon2Wrapper;
import frc.team2872.sensors.ThreadedPIDController;
import frc.team2872.HelperFunctions;
import frc.team2872.auton.AutonRecorder;
import frc.team2872.auton.AutonRecorderData;
import frc.team2872.auton.XboxControllerState;
import static frc.team2872.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  public static double driveAngle = 0;
  public static double driveMag = 0;
  public static double rotMag = 0;


  // Controllers
  private XboxController driveController;
  private XboxController operatorController;
 //private Joystick operatorJOYSTICKController;

  // Drive Objects
  public static SwerveDrive swerveDrive;
  public static Pigeon2Wrapper gyro;
  public static ThreadedPIDController gyroPIDController;

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Selected Auton String
  private String selectedAutonType;
  private String selectedAutonMode;

  // Shuffleboard Choosers
  private SendableChooser<Boolean> fieldOrientationChooser;
  private SendableChooser<String> autonModeChooser, autonTypeChooser, PPPath;

  // Global Variables
  private boolean fieldOrientationChosen;

  // Edge Triggers
  private boolean zeroEdgeTrigger;
  public static boolean quickTurning;

  public static final Flywheel flywheel = new Flywheel();
  public static final Indexer indexer = new Indexer();
  public static final Intake intake = new Intake();
  public static final LeadScrew leadscrew = new LeadScrew();
  public static final LimeLight limelight = new LimeLight();
  public static final LED blinkin = new LED();
  public static final frc.robot.Climbers climbers = new frc.robot.Climbers();

  /** Shooter Commands **/
  private final ShootSpeakerCommand SSC = new ShootSpeakerCommand(flywheel, indexer);
  private final IntakeFromShooterCommand IFS = new IntakeFromShooterCommand(flywheel, indexer);
  private final ShootAmpCommand SAC = new ShootAmpCommand(flywheel, indexer);
  private final ShootTrapCommand STC = new ShootTrapCommand(flywheel, indexer);
  private final IntakeFromGroundCommand IC = new IntakeFromGroundCommand(intake, indexer);
  private final DriveToPointCommand DPC = new DriveToPointCommand(new Pose2d(new Translation2d(2.5, 2.5), Rotation2d.fromDegrees(-160)));
  private final AlignHorizontallyCommand AHC = new AlignHorizontallyCommand();
  private final OuttakeCommand OC = new OuttakeCommand(intake, indexer);
  //private final AutoAimCommand AAC = new AutoAimCommand(leadscrew);

  /** Button Numbers **/
  int A = 1;
  int B = 2;
  int X = 3;
  int Y = 4;
  int leftTop = 5;
  int rightTop = 6;
  int leftMid = 7;
  int rightMid = 8;
  int leftJoystickPressed = 9;
  int rightJoystickPressed = 10;
  int leftTriggerAxis = 2;
  int rightTriggerAxis = 3;


  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    // Sensors
    gyro = new Pigeon2Wrapper(GYRO_CAN_ID);
    gyro.reset();

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(gyro.asSupplier(), GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX, true);
    gyroPIDController.start();

    // Swerve Drivetrain Init
    swerveDrive = new SwerveDrive(leftRear_Unit_Config, leftFront_Unit_Config, rightFront_Unit_Config, rightRear_Unit_Config);


    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = "Playback";
    autonTypeChooser= new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auto", "Playback");
    autonTypeChooser.addOption("Record Auto", "Record");
    autonTypeChooser.addOption("PathPlanner", "PathPlanner");
    SmartDashboard.putData("Auto Type: Recorded/PathPlanner", autonTypeChooser);

    //PathPlanner Auto Chooser
    PPPath = new SendableChooser<>();

    // Field Orientation Chooser
    fieldOrientationChooser = new SendableChooser<>();
    fieldOrientationChooser.setDefaultOption("Pick an option", null);
    fieldOrientationChooser.addOption("0\u00B0", false);
    fieldOrientationChooser.addOption("180\u00B0", true);
    SmartDashboard.putData("Field Orientation Chooser", fieldOrientationChooser);
    SmartDashboard.putBoolean("Flipped", false);

    // Auton Routine init
    selectedAutonMode = DEFAULT_AUTON;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(DEFAULT_AUTON, DEFAULT_AUTON);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auto Routines", autonModeChooser);

    // Global Variable Init
    fieldOrientationChosen = false;

    // Edge Trigger Init
    zeroEdgeTrigger = false;

    JoystickButton shootSpeaker = new JoystickButton(operatorController, B);
    shootSpeaker.whileTrue(SSC);
    JoystickButton intakeFromShooter = new JoystickButton(operatorController, X);
    intakeFromShooter.whileTrue(IFS);
    JoystickButton shootAmp = new JoystickButton(operatorController, A);
    shootAmp.whileTrue(SAC);
    JoystickButton shootTrap = new JoystickButton(operatorController, Y);
    shootTrap.whileTrue(STC);
    JoystickButton intake = new JoystickButton(operatorController, leftTop);
    intake.whileTrue(IC);
    JoystickButton alignHoriz = new JoystickButton(driveController, rightTop);
    alignHoriz.whileTrue(AHC);
    JoystickButton outtake = new JoystickButton(operatorController, rightTop);
    outtake.whileTrue(OC);

    //JoystickButton autoaim = new JoystickButton(operatorController, rightTrig);
    //autoaim.whileTrue(AAC);
    //JoystickButton alignSpeaker = new JoystickButton(operatorController, leftTrig);
    //alignSpeaker.whileTrue(AWS);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Gyro Value
    final double gyroValue = gyroPIDController.getSensorValue();

    // Field Orientation Chooser
    final Boolean fieldOrientationBoolean = fieldOrientationChooser.getSelected();
    // Until a valid option is choosen leave the gyro orientation alone
    if (fieldOrientationBoolean != null && !fieldOrientationChosen) {
      // Prevents the field orientation from being changed until a reboot
      fieldOrientationChosen = true;
      // If the selected option is true then flip the orientation 180 degrees
      if (fieldOrientationBoolean.booleanValue()) {
        gyro.setYaw(Normalize_Gryo_Value(gyroValue + 180));
        SmartDashboard.putBoolean("Flipped", true);
      } else {
        // Else don't flip the field orientation
        SmartDashboard.putBoolean("Flipped", false);
      }
    }

    // Driver Controller Info
    double leftStickX, leftStickY, rightStickX, leftStickAngle, leftStickMagnitude, fieldCorrectedAngle;
    if (driveController.isConnected()) {
      // Get the controller values
      leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getLeftX());
      leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveController.getLeftY());
      rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

      // Calculate the left stick angle and magnitude
      leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
      leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
      if (leftStickMagnitude > 1.0) {
        leftStickMagnitude = 1;
      }

      fieldCorrectedAngle = Normalize_Gryo_Value(leftStickAngle - gyroValue);
    } else {
      leftStickX = 0;
      leftStickY = 0;
      leftStickAngle = 0;
      leftStickMagnitude = 0;
      rightStickX = 0;
      fieldCorrectedAngle = 0;
    }

    // Put the values on Shuffleboard
    SmartDashboard.putString("Gyro", String.format("%.2f\u00B0", gyroValue));
    if (DEBUG) {
      // Operator Controller Values
      SmartDashboard.putString("Left Stick Y", String.format("%.2f", leftStickY));
      SmartDashboard.putString("Right Stick X", String.format("%.2f", rightStickX));
      SmartDashboard.putString("Left Stick Angle", String.format("%.2f\u00B0", leftStickAngle));
      SmartDashboard.putString("Left Stick Velocity", String.format("%.2f", leftStickMagnitude));
      SmartDashboard.putString("Field Adjusted Angle", String.format("%.2f\u00B0", fieldCorrectedAngle));
      swerveDrive.SmartDashboard();
    }

    // Calibrate Swerve Drive
    final boolean zeroTrigger = driveController.getBackButton() && driveController.getStartButton() && isDisabled();
    if (zeroTrigger && !zeroEdgeTrigger) {
      // Zero the swerve units
      swerveDrive.zero();
      System.out.println("Swerve Drive has been calibrated!");
    }

    zeroEdgeTrigger = zeroTrigger;

    //swerveDrive.updatePose();


    SmartDashboard.putNumber("Top Flywheel Speed", flywheel.getTopRPM());
    SmartDashboard.putNumber("Bottom Flywheel Speed", flywheel.getBotRPM());
    SmartDashboard.putNumber("Top Flywheel Absolute", flywheel.getTopAFlywheel());

    //SmartDashboard.putNumberArray("Botpose LL", limelight.getBOTPOSE());

    SmartDashboard.putNumber("right Climb", climbers.getPosition("right"));
    SmartDashboard.putNumber("left Climb", climbers.getPosition("left"));
    SmartDashboard.putNumber("leadScrew Position", leadscrew.getPosition());
  }

  @Override
  public void autonomousInit() {
    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType.equals("Record")) {
      playbackData = null;
    }
    else if (selectedAutonType.equals("Playback")) {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          disabledInit();
          playbackData = null;
          break;
        default:
          AutonRecorder.loadFromFile(autonPlaybackQueue, selectedAutonMode);
          playbackData = autonPlaybackQueue.poll();
          robotControlsInit();
          break;
      }
    }
    else { //pathplanner
      SmartDashboard.putData("PathPlanner Auto", PPPath);
      //TODO: Do pathplanner things

    }
    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        disabledPeriodic();
        break;
      default:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the controller states
          final XboxControllerState driveControllerState = playbackData.getDriveControllerState();
          final XboxControllerState operatorControllerState = playbackData.getOperatorControllerState();

          // Robot drive controls
          robotControlsPeriodic(driveControllerState, operatorControllerState);

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        } else {
          selectedAutonMode = DEFAULT_AUTON;
          disabledInit();
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType.equals("Record");

    // Reset the robot controls
    robotControlsInit();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();

    quickTurning = false;
  }

  @Override
  public void teleopPeriodic() {
    // Get the controller states
    final XboxControllerState driveControllerState = new XboxControllerState(driveController);
    final XboxControllerState operatorControllerState = new XboxControllerState(operatorController);

    // Robot drive controls
    robotControlsPeriodic(driveControllerState, operatorControllerState);

    // Auton Recording
    final double autonTimeStamp = Timer.getFPGATimestamp() - autonStartTime;
    if (saveNewAuton && autonTimeStamp <= 15) {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder.addNewData(new AutonRecorderData(autonTimeStamp, driveControllerState, operatorControllerState));
          break;
      }
    }

    /*
    if(driveController.getRawButton(X)){
      CommandScheduler.getInstance().schedule(swerveDrive.followPathCommand(new Pose2d(5, 1, new Rotation2d(0))));
    }
     */
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    gyroPIDController.disablePID();

    // Once auton recording is done, save the data to a file, if there is any
    if (saveNewAuton) {
      saveNewAuton = false;
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          autonRecorder.saveToFile(selectedAutonMode);
          break;
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    // Do Nothing
  }

  @Override
  public void testInit() {
    // Do Nothing
  }

  @Override
  public void testPeriodic() {
    // Do Nothing
  }

  private void robotControlsInit() {
    // Reset the drive controller
    swerveDrive.driveInit();
    gyroPIDController.enablePID();
    gyroPIDController.updateSensorLockValue();
  }

  private void robotControlsPeriodic(final XboxControllerState driveControllerState, final XboxControllerState operatorControllerState) {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
    double leftStickMagnitude;
    leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
    if (leftStickMagnitude > 1.0) {
      leftStickMagnitude = 1;
    }

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

    // Drive Controls
    final boolean precisionMode = driveControllerState.getLeftBumper();
    SmartDashboard.putBoolean("precisionMode", precisionMode);
    if (rightStickX >= 0.05 || rightStickX <= -0.05) {
      // Manual turning

      quickTurning = false;

      gyroPIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, precisionMode);
      SmartDashboard.putNumber("angularVelocityZ", Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()));


    } else {
      // Normal gyro locking

      SmartDashboard.putNumber("averageDriveSpeed", swerveDrive.getAverageDriveSpeed());
      //(swerveDrive.getAverageDriveSpeed() > 0.8 || swerveDrive.getAverageDriveSpeed() < -0.8)

      //SmartDashboard.putNumber("angularVelocityY", Math.abs(gyro.getAngularVelocityYDevice().getValueAsDouble()));
      //SmartDashboard.putNumber("angularVelocityX", Math.abs(gyro.getAngularVelocityXDevice().getValueAsDouble()));
      //SmartDashboard.putNumber("angularVelocityZ", Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()));

      if(!quickTurning && Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 15){
        //gyroPIDController.disablePID();
        gyroPIDController.updateSensorLockValue();
      }
      else{
        gyroPIDController.enablePID();
      }
      /*
      if(swerveDrive.getAverageDriveSpeed() > 0){
        gyroPIDController.updateSensorLockValue();
      }
       */
      gyroPIDController.enablePID();

      // Quick Turning
      if (driveControllerState.getPOV() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
        quickTurning = true;
      } /* else if (driveControllerState.getYButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(0);
      } else if (driveControllerState.getBButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(90);
      } else if (driveControllerState.getAButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(180);
      } else if (driveControllerState.getXButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(-90);
      } */

      SmartDashboard.putNumber("gyroSensorLockValue", gyroPIDController.getSensorLockValue());
      SmartDashboard.putNumber("gyroPIDValue", gyroPIDController.getPIDValue());
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ?  gyroPIDController.getPIDValue() : 0, precisionMode);

    }
    // Operator Controls
    if (operatorControllerState.getBButton()) {
      SSC.schedule();
    }else{
      CommandScheduler.getInstance().cancel(SSC);
    }

    if (operatorControllerState.getLeftBumper()) {
      IC.schedule();
    }else{
      CommandScheduler.getInstance().cancel(IC);
    }

    /**move lead screw manually**/
    if (operatorController.getPOV() == 0){
      leadscrew.moveShooterUp(-0.3);
    } else if(operatorController.getPOV() == 180){
      leadscrew.moveShooterDown(0.4);
    } else {
      leadscrew.stop();
    }

    /**move climbers manually**/
    // LEFT CLIMBER
    if (operatorController.getRawAxis(1) < -0.04){ // left joystick up
      climbers.raiseLeftClimber();
    } else if (operatorController.getRawAxis(1) > 0.04){ // left joystick down
      climbers.lowerLeftClimber();
    } else {
      climbers.stopLeftClimber();
    }

    // RIGHT CLIMBER
    if (operatorController.getRawAxis(5) < -0.04){ // right joystick up
      climbers.raiseRightClimber();
    } else if (operatorController.getRawAxis(5) > 0.04){ // right joystick down
      climbers.lowerRightClimber();
    } else {
      climbers.stopRightClimber();
    }




    //reset gyro to 0
    if (driveController.getRawButton(3)){
      gyroPIDController.disablePID();
      gyro.reset();
      gyroPIDController.enablePID();
    }
  }
}
