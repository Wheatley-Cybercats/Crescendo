
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;

import static com.revrobotics.CANSparkBase.ControlType.kPosition;
import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkBase.FaultID.kSensorFault;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static frc.robot.Constants.Swerve.swerveCANcoderConfig;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;
    private AbsoluteEncoder absoluteEncoder;


    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  kBrushless);
        configDriveMotor();

        /* Angle Encoder Config */

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


        // lastAngle = getState().angle;
    }


    private void configEncoders()
    {
        // absolute encoder
        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);


        relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);


        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);

        absoluteEncoder = mAngleMotor.getAbsoluteEncoder(kDutyCycle);


        synchronizeEncoders();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
    }

    private void configAngleMotor()
    {
        //TODO: Most likely to be wrong here too
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(Constants.Swerve.angleKP, 0);
        controller.setI(Constants.Swerve.angleKI,0);
        controller.setD(Constants.Swerve.angleKD,0);
        controller.setFF(Constants.Swerve.angleKFF,0);
        controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        mAngleMotor.setClosedLoopRampRate(Constants.Swerve.angleRampRate);
    }

    private void configDriveMotor()
    {
        //TODO: Most likely to be wrong here too
        mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(Constants.Swerve.driveKP,0);
        controller.setI(Constants.Swerve.driveKI,0);
        controller.setD(Constants.Swerve.driveKD,0);
        controller.setFF(Constants.Swerve.driveKFF,0);
        controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {


        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);

        if(mDriveMotor.getFault(kSensorFault))
        {
            DriverStation.reportWarning("oh no! Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(kSensorFault))
        {
            DriverStation.reportWarning("oh no! Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, kVelocity, 0);

    }

    private void setAngle(SwerveModuleState desiredState)
    {
        //TODO: Most likely to be wrong here too
        if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle;
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        SparkPIDController controller = mAngleMotor.getPIDController();

        double degReference = angle.getDegrees();



        controller.setReference (degReference, kPosition, 0);

    }



    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }
    public double getAbsAngle(){return absoluteEncoder.getPosition();}

    public Rotation2d getCanCoder()
    {

        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    public void synchronizeEncoders()
    {

        double absolutePosition =getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }



    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega()
    {
        return angleEncoder.getVelocity().getValueAsDouble()/360;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }
}
