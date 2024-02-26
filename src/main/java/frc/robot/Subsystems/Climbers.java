package frc.robot;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class Climbers extends SubsystemBase {
    TalonFX leftClimbMotor = new TalonFX(RobotProperties.Climber.LEFT_CLIMB_MOTOR_ID);
    TalonFX rightClimbMotor = new TalonFX(RobotProperties.Climber.RIGHT_CLIMB_MOTOR_ID);

    public Climbers() {
        leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        leftClimbMotor.setPosition(0);
        rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimbMotor.setPosition(0);
    }
    public void raiseBothClimbers(){
        leftClimbMotor.set(RobotProperties.Climber.upSpeed);
        rightClimbMotor.set(RobotProperties.Climber.upSpeed);
    }
    public void raiseLeftClimber(){
        leftClimbMotor.set(RobotProperties.Climber.upSpeed);
    }
    public void raiseRightClimber(){
        rightClimbMotor.set(RobotProperties.Climber.upSpeed);
    }
    public double getPosition(String climberSide){
        if (climberSide.equals("left")) {
            return leftClimbMotor.getPosition().getValueAsDouble();
        }
        return rightClimbMotor.getPosition().getValueAsDouble(); //"right"
    }
    public void lowerLeftClimber(){
        leftClimbMotor.set(RobotProperties.Climber.downSpeed);

    }
    public void lowerRightClimber(){
        rightClimbMotor.set(RobotProperties.Climber.downSpeed);
    }
    public void lowerBothClimbers() {
        leftClimbMotor.set(RobotProperties.Climber.downSpeed);
        rightClimbMotor.set(RobotProperties.Climber.downSpeed);
    }
    public void stopLeftClimber(){
        leftClimbMotor.stopMotor();
    }
    public void stopRightClimber(){
        rightClimbMotor.stopMotor();
    }
    public void stopClimbers(){
        rightClimbMotor.stopMotor();
        leftClimbMotor.stopMotor();
    }

}
