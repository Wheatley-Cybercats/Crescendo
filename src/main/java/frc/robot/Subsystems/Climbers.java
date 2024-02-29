package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class Climbers extends SubsystemBase {
    TalonFX leftClimbMotor = new TalonFX(RobotProperties.ClimberProperties.LEFT_CLIMB_MOTOR_ID);
    TalonFX rightClimbMotor = new TalonFX(RobotProperties.ClimberProperties.RIGHT_CLIMB_MOTOR_ID);

    public Climbers() {
        leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        leftClimbMotor.setPosition(0);
        rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimbMotor.setPosition(0);
    }
    public void raiseBothClimbers(){
        leftClimbMotor.set(RobotProperties.ClimberProperties.upSpeed);
        rightClimbMotor.set(RobotProperties.ClimberProperties.upSpeed);
    }
    public void raiseLeftClimber(){
        leftClimbMotor.set(RobotProperties.ClimberProperties.upSpeed);
    }
    public void raiseRightClimber(){
        rightClimbMotor.set(RobotProperties.ClimberProperties.upSpeed);
    }
    public double getPosition(String climberSide){
        if (climberSide.equals("left")) {
            return leftClimbMotor.getPosition().getValueAsDouble();
        }
        return rightClimbMotor.getPosition().getValueAsDouble(); //"right"
    }
    public void lowerLeftClimber(){
        leftClimbMotor.set(RobotProperties.ClimberProperties.downSpeed);

    }
    public void lowerRightClimber(){
        rightClimbMotor.set(RobotProperties.ClimberProperties.downSpeed);
    }
    public void lowerBothClimbers() {
        leftClimbMotor.set(RobotProperties.ClimberProperties.downSpeed);
        rightClimbMotor.set(RobotProperties.ClimberProperties.downSpeed);
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
