package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotProperties;

public class Climber extends SubsystemBase {
    /*
    TalonFX leftClimbMotor = new TalonFX(RobotProperties.Climber.LEFT_CLIMB_MOTOR_ID);
    TalonFX rightClimbMotor = new TalonFX(RobotProperties.Climber.RIGHT_CLIMB_MOTOR_ID);

    public Climber() {
        leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
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
    public void stopClimbers(){
        rightClimbMotor.stopMotor();
        leftClimbMotor.stopMotor();
    }

     */
}

