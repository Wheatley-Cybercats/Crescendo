package frc.robot.Subsystems;


import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class Intake extends SubsystemBase {

    CANSparkMax intakeMotor1 = new CANSparkMax(RobotProperties.IntakeProperties.INTAKE_MOTOR1_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax intakeMotor2 = new CANSparkMax(RobotProperties.IntakeProperties.INTAKE_MOTOR2_ID, CANSparkLowLevel.MotorType.kBrushless);

    public Intake() {
       intakeMotor1.setIdleMode(CANSparkFlex.IdleMode.kBrake);
       intakeMotor2.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    public void setSpeed(double speed){
        intakeMotor1.set(speed);
        intakeMotor2.set(speed);
    }
    public void stop(){
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }
}