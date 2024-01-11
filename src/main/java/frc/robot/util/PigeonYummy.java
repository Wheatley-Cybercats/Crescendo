package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PigeonYummy extends Pigeon2{

    private static PigeonYummy instance;
    public static double zeroHeading;
    public static double zeroAngle;

    private PigeonYummy(){
        super(1);
        resetGyro();
        zeroHeading = getNavHeading() + ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? 90 : -90);
        zeroAngle = getNavAngle();

    }

    public static PigeonYummy getInstance(){
        if (instance == null){
            instance = new PigeonYummy();
        }
        return instance;
    }

    public double getNavHeading(){
        return getYaw();
    }

    public double getNavAngle(){
        return getYaw();
    }

    public double getHeading() {
        return Math.IEEEremainder(-getNavAngle(), 360) - ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? 180 : 0);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro(){
        setYaw(0);
    }

}
