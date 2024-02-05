package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class Indexer extends SubsystemBase {

    CANSparkMax indexerMotor = new CANSparkMax(RobotProperties.IndexerProperties.INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    public Indexer() {
        indexerMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }
    public void setSpeed(double speed){
        indexerMotor.set(speed);
    }
    public void stop(){
        indexerMotor.set(0);
    }
}