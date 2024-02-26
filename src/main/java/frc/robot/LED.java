package frc.robot;


import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    /*
     * The Blinkin takes a PWM signal from 1000-2000us
     * This is identical to a SparkMax motor controller
     * -1 corresponds to 1000us
     * 0 corresponds to 1500us
     * 1 corresponds to 2000us
     */

    private static Spark blinkin = new Spark(1);
    public LED(){
        solid_green();
    }
    public void set (double num){
        if ((num >= -1.0) && (num <= 1.0)){
            blinkin.set(num);
        }
    }
    public void rainbow_rainbow(){
        set(-0.99);
    }
    public void rainbow_party(){
        set(-0.97);
    }
    public void wave_rainbow(){
        set(-0.45);
    }
    public void wave_forest(){
        set(-0.37);
    }
    public void confetti(){
        set(-0.87);
    }
    public void solid_dark_red(){
        set(0.59);
    }
    public void solid_red(){
        set(0.61);
    }
    public void solid_green(){
        set(0.77);
    }
    public void allianceColor(){
        boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        if (isRed){
            Robot.blinkin.set(-0.01);
        } else {
            Robot.blinkin.set(0.19);
        }
    }



}

