package frc.robot;

// Team 3171 Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team2872.drive.SwerveUnitConfig;
import frc.team2872.drive.SwerveUnitConfig.ENCODER_TYPE;
import frc.team2872.drive.SwerveUnitConfig.MOTOR_TYPE;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /**
         * Debug Info
         **/
        public static final boolean DEBUG = true;
        public static final String PID_LOG_ADDRESS = "10.28.72.201";

        /**
         * Drive Variables
         **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = 1, MAX_ROTATION_SPEED = .9;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = false;

        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;
        public static final boolean USE_ODOMETRY_DRIVE_TO_POINT = true;
        public static final Pose2d redSpeakerPose = new Pose2d(16.178, 5.45, Rotation2d.fromDegrees(180));
        public static final Pose2d blueSpeakerPose = new Pose2d(0, 1, Rotation2d.fromDegrees(0));;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig leftRear_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 52, true,
                MOTOR_TYPE.REV, 51, false, ENCODER_TYPE.CTRE, 12, false, "The CANivore", true,
                new Translation2d(-0.318, -0.318));
        public static final SwerveUnitConfig leftFront_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 62, true,
                MOTOR_TYPE.REV, 61, false, ENCODER_TYPE.CTRE, 10, false, "The CANivore", true,
                new Translation2d(-0.318, 0.318));
        public static final SwerveUnitConfig rightFront_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 50, true,
                MOTOR_TYPE.REV, 49, false, ENCODER_TYPE.CTRE, 11, false,"The CANivore", true,
                new Translation2d(0.318, 0.318));
        public static final SwerveUnitConfig rightRear_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 55, true,
                MOTOR_TYPE.REV, 56, false, ENCODER_TYPE.CTRE, 13, false,"The CANivore", true,
                new Translation2d(0.318, -0.318));



        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 0;


        /** FLYWHEEL Constants **/
        public static final class FlywheelProperties {
                public static final int TOP_FLYWHEEL_MOTOR_ID = 39;
                public static final int BOT_FLYWHEEL_MOTOR_ID = 40;
                public static final int topFlywheelKp = 0;
                public static final int topFlywheelKi = 0;
                public static final int topFlywheelKd = 0;
                public static final int botFlywheelKp = 0;
                public static final int botFlywheelKi = 0;
                public static final int botFlywheelKd = 0;
                public static final int topFlywheelKs = 0;
                public static final int topFlywheelKv = 0;
                public static final int botFlywheelKs = 0;
                public static final int botFlywheelKv = 0;
        }

        /** INDEXER Constants **/
        public static final class IndexerProperties {
                public static final int INDEXER_MOTOR_ID = 9;
                public static final double shootingSpeed = -.17;
                public static final double indexerIntakingSpeed = .15;
                public static final int BEAM_BREAK_DIO_PORT = 0;

        }

        /** Lead Screw Constants **/
        public static final int LEAD_SCREW_MOTOR_ID = 10;
        public static final double AMP_ANGLE = 91.5;
        public static final double PODIUM_ANGLE = 36;
        public static final double WING_ANGLE = 21; //17; //when front of bumpers are lined up with outer edge of stage
        public static final double SUBWOOFER_ANGLE = 115;
        public static final double THRU_BORE_ENCODER_PORT = 4;

        // y = 250\left(0.993624\right)^{x}

        /** Intake Constants **/
        public static final class IntakeProperties {
                public static final int TOP_INTAKE_MOTOR_ID = 5;
                public static final int BOT_INTAKE_MOTOR_ID = 4;
                public static final double intakeIntakingSpeed = .6; //.6;
        }
        /** Climb Constants **/
        public static final class Climber {
                public static final int LEFT_CLIMB_MOTOR_ID = 20;
                public static final int RIGHT_CLIMB_MOTOR_ID = 21;
                public static final double upSpeed = -.75;
                public static final double downSpeed = .6;

        }
        /** AprilTag/Limelight constants **/
        public static final double angleOfLL = 30;
        public static double LLmultiplier = 1;
        /** PID Properties **/

        //-0.00005, 0, 0
        //.013, 0.00075, 0.00075
        public static final double GYRO_KP = 0.007, GYRO_KI = 0, GYRO_KD = -0.02, GYRO_MIN = -0.5, GYRO_MAX = 0.5;//-0.5, 0.000008
        public static final double SLEW_KP = -0.0072, SLEW_KI = -0, SLEW_KD = 0, SLEW_KF = 0, SLEW_PID_MIN = -.7, SLEW_PID_MAX = .5;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = {"4 Note (3 In-Wing)", "4 Note (2 In-Wing 1 Midline)", "Test 3"};
}
