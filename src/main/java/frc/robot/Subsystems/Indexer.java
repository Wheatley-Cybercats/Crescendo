package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotProperties;

public class Indexer extends SubsystemBase {


    CANSparkMax indexerMotor = new CANSparkMax(RobotProperties.IndexerProperties.INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    DigitalInput beamBreak; //shoudl be here

    public Indexer() {
        indexerMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        beamBreak = new DigitalInput(RobotProperties.IndexerProperties.BEAM_BREAK_DIO_PORT);
    }
    public void setSpeed(double speed){
        indexerMotor.set(speed);
    }
    public void stop(){
        indexerMotor.set(0);
    }
    public boolean getBeamBreakState(){
        return beamBreak.get();
    }
}