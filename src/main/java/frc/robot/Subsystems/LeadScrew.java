package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotProperties;

public class LeadScrew extends SubsystemBase {

    TalonFX leadScrewMotor = new TalonFX(RobotProperties.LEAD_SCREW_MOTOR_ID, "The CANivore");
    //DutyCycleEncoder thruBoreEncoder = new DutyCycleEncoder(RobotProperties.THRU_BORE_ENCODER_PORT);

    public LeadScrew() {
        leadScrewMotor.setNeutralMode(NeutralModeValue.Coast);
        //leadScrewMotor.setPosition(0); //sets motor position to 0 whenever robot is initiated
    }
    public void moveShooterUp(double speed){
        leadScrewMotor.set(-speed);
        //thruBoreEncoder.getAbsolutePosition();
    }
    public void moveShooterDown(double speed){
        leadScrewMotor.set(-speed);
    }
    public boolean atPosition(double position){
        if (getPosition() - position < 1.2 && getPosition() - position > -1.2){ //the motor should stop whenever it is at a specific position
            return true;
        }
        return false;
    }
    public void moveToPosition(double position){ //if starting at top = 0, moving the shooter down goes negative
        if (leadScrewMotor.getPosition().getValueAsDouble() > position){ //less negative: current position is higher than desired position
            moveShooterDown(Math.sqrt(Math.abs(leadScrewMotor.getPosition().getValueAsDouble() - position))/1.5);
        } else if (leadScrewMotor.getPosition().getValueAsDouble() < position){ //current position is lower than desired
            moveShooterUp(-Math.sqrt(Math.abs(leadScrewMotor.getPosition().getValueAsDouble() - position))/4.5);
        } else {
            stop();
        }
    }
    public double getPosition(){
       return leadScrewMotor.getPosition().getValueAsDouble();
    }
    public void setPosition(double position){
        leadScrewMotor.setPosition(position);
    }
    public void stop() {
        leadScrewMotor.stopMotor();
    }
     }
