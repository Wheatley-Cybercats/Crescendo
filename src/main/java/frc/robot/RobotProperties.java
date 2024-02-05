package frc.robot;

// Team 3171 Imports
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
        public static final String PID_LOG_ADDRESS = "10.31.71.201";

        /**
         * Drive Variables
         **/
        public static final boolean FIELD_ORIENTED_SWERVE = true;
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double MAX_DRIVE_SPEED = .8, MAX_ROTATION_SPEED = .9;
        public static final boolean PINWHEEL_ZERO_ORIENTATION = false;

        public static final boolean SWERVE_UNIT_ORIENTATION_OPTIMIZATION = true;
        public static final boolean USE_ODOMETRY_DRIVE_TO_POINT = true;

        /**
         * Swerve Unit Configuration
         **/
        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig leftRear_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 52, true,
                MOTOR_TYPE.REV, 51, false, ENCODER_TYPE.CTRE, 12, false);
        public static final SwerveUnitConfig leftFront_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 62, true,
                MOTOR_TYPE.REV, 61, false, ENCODER_TYPE.CTRE, 10, false);
        public static final SwerveUnitConfig rightFront_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 50, true,
                MOTOR_TYPE.REV, 49, false, ENCODER_TYPE.CTRE, 11, false);
        public static final SwerveUnitConfig rightRear_Unit_Config = new SwerveUnitConfig(MOTOR_TYPE.REV, 55, true,
                MOTOR_TYPE.REV, 56, false, ENCODER_TYPE.CTRE, 13, false);



        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 0;

        /** FLYWHEEL Constants **/
        public static final class FlywheelProperties {
                public static final int TOP_FLYWHEEL_MOTOR_ID = 39;
                public static final int BOT_FLYWHEEL_MOTOR_ID = 43;
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
                public static final double indexerIntakingSpeed = .1;
        }

        /** Lead Screw Constants **/
        public static final int LEAD_SCREW_MOTOR_ID = 4;

        /** Intake Constants **/
        public static final class IntakeProperties {
                public static final int INTAKE_MOTOR1_ID = 5;
                public static final int INTAKE_MOTOR2_ID = 6;
                public static final double intakeIntakingSpeed = .4;
        }
        /** AprilTag/Limelight constants **/
        public static final double angleOfLL = 30;
        public static final double heightOfTarget = 70;//height of speaker
        public static double LLmultiplier = 1;
        /** PID Properties **/
        public static final double GYRO_KP = -0.00005, GYRO_KI = 0, GYRO_KD = 0, GYRO_MIN = -5, GYRO_MAX = 5;
        public static final double SLEW_KP = -0.0065, SLEW_KI = -0.004, SLEW_KD = 0.0005, SLEW_KF = 0, SLEW_PID_MIN = -.7, SLEW_PID_MAX = .5;//-1 and 1

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = {"4 Note (3 In-Wing)", "4 Note (2 In-Wing 1 Midline)", "Test 3"};
}
