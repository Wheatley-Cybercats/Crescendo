package frc.robot.Subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class Flywheel extends SubsystemBase {

    CANSparkFlex topFlywheelMotor = new CANSparkFlex(RobotProperties.FlywheelProperties.TOP_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkFlex botFlywheelMotor = new CANSparkFlex(RobotProperties.FlywheelProperties.BOT_FLYWHEEL_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder topFlywheelEncoder;
    private RelativeEncoder botFlywheelEncoder;
    private AbsoluteEncoder topFlywheelAEnconder;
    private AbsoluteEncoder botFlywheelAEncoder;
    private final BangBangController topFlywheelBangBangController = new BangBangController();
    private final BangBangController botFlywheelBangBangController = new BangBangController();
    private final PIDController topFlywheelPIDController = new PIDController(RobotProperties.FlywheelProperties.topFlywheelKp,RobotProperties.FlywheelProperties.topFlywheelKi,RobotProperties.FlywheelProperties.topFlywheelKd);
    private final PIDController botFlywheelPIDController = new PIDController(RobotProperties.FlywheelProperties.botFlywheelKp, RobotProperties.FlywheelProperties.botFlywheelKi, RobotProperties.FlywheelProperties.botFlywheelKd);
    private final SimpleMotorFeedforward topflywheelFeedForward = new SimpleMotorFeedforward(RobotProperties.FlywheelProperties.topFlywheelKs, RobotProperties.FlywheelProperties.topFlywheelKv);
    private final SimpleMotorFeedforward bottomflywheelFeedForward = new SimpleMotorFeedforward(RobotProperties.FlywheelProperties.botFlywheelKs, RobotProperties.FlywheelProperties.botFlywheelKv);

    public Flywheel() {
        topFlywheelMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        topFlywheelEncoder = topFlywheelMotor.getEncoder();
        topFlywheelAEnconder = topFlywheelMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        topFlywheelEncoder.setPosition(0);
        botFlywheelMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        botFlywheelAEncoder = botFlywheelMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        botFlywheelEncoder = botFlywheelMotor.getEncoder();
        botFlywheelEncoder.setPosition(0);

        topFlywheelMotor.setInverted(false);
        botFlywheelMotor.setInverted(false);
    }

    public void setTopFlywheelMotorVolt(double topWheelVolt) {
        topFlywheelMotor.setVoltage(topWheelVolt);
    }
    public void setBotFlywheelMotorVolt(double botWheelVolt) {
        botFlywheelMotor.setVoltage(botWheelVolt);
    }
    public double getTopEncoderValue(){
        return topFlywheelEncoder.getPosition();
    }
    public double getBotEncoderValue(){
        return botFlywheelEncoder.getPosition();
    }
    public double getTopRPM(){
        return topFlywheelMotor.getEncoder().getVelocity();
    }
    public double getBotRPM(){
        return botFlywheelMotor.getEncoder().getVelocity();
    }

    public double getTopAFlywheel(){
        return topFlywheelAEnconder.getPosition();
    }
    public void setTopMotor(double speed){
        topFlywheelMotor.set(speed);
    }
    public void setBotMotor(double speed){
        botFlywheelMotor.set(speed);
    }
    public boolean topflywheelAtSpeed(){
        //return (topFlywheelPIDController.atSetpoint());
        return topFlywheelMotor.getEncoder().getVelocity() > 4100;
    }
    public boolean botflywheelAtSpeed(){
        //return(botFlywheelPIDController.atSetpoint());
        return botFlywheelMotor.getEncoder().getVelocity() < -4100;
    }
    public boolean topflywheelAtAmpSpeed(){
        //return (topFlywheelPIDController.atSetpoint());
        return topFlywheelMotor.getEncoder().getVelocity() > 380;
    }
    public boolean botflywheelAtAmpSpeed(){
        //return(botFlywheelPIDController.atSetpoint());
        return botFlywheelMotor.getEncoder().getVelocity() < -1600;
    }
    public boolean topflywheelAtTrapSpeed(){
        //return (topFlywheelPIDController.atSetpoint());
        return topFlywheelMotor.getEncoder().getVelocity() > 2400;
    }
    public boolean botflywheelAtTrapSpeed(){
        //return(botFlywheelPIDController.atSetpoint());
        return botFlywheelMotor.getEncoder().getVelocity() < -2400;
    }
    public void resetPID(){
        topFlywheelPIDController.setSetpoint(0);
        botFlywheelPIDController.setSetpoint(0);
        topFlywheelBangBangController.setSetpoint(0);
        botFlywheelBangBangController.setSetpoint(0);
    }
    public void stop(){
        topFlywheelMotor.stopMotor();
        botFlywheelMotor.stopMotor();
    }
}

