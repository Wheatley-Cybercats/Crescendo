package frc.robot.Subsystems.led;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Blinkin extends SubsystemBase {
  /*
   * The Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor controller
   * -1 corresponds to 1000us
   * 0 corresponds to 1500us
   * 1 corresponds to 2000us
   */
  private Spark blinkin = new Spark(9);

  public Blinkin() {
    solid_dark_red();
  }

  public void set(double num) {
    if ((num >= -1.0) && (num <= 1.0)) {
      blinkin.set(num);
    }
  }

  public void rainbow_rainbow() {
    set(-0.99);
  }

  public void rainbow_party() {
    set(-0.97);
  }

  public void wave_rainbow() {
    set(-0.45);
  }

  public void wave_forest() {
    set(-0.37);
  }

  public void confetti() {
    set(-0.87);
  }

  public void solid_dark_red() {
    set(0.59);
  }

  public void solid_orange() {
    set(0.65);
  }

  public void solid_red() {
    set(0.61);
  }

  public void solid_green() {
    set(0.77);
  }

  public void heartbeat_color1() {
    set(0.05);
  }

  public void shot_color1() {
    set(0.13);
  }

  public void strobe_gold() {
    set(-0.07);
  }

  public void allianceColor() {
    boolean isRed =
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("IsRedAlliance")
            .getBoolean(true);
    if (isRed) {
      this.set(0.01);
    } else {
      this.set(0.21);
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED", this.blinkin.get());
  }
}
