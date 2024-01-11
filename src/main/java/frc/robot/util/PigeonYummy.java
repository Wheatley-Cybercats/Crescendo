package frc.robot.util;
;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PigeonYummy extends Pigeon2 {

    private static PigeonYummy instance;
    public static double zeroHeading;
    public static double zeroAngle;

    private PigeonYummy(){
        super(0);
        resetGyro();
        if(DriverStation.getAlliance().isPresent()) {
            zeroHeading = getNavHeading() + ((DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) ? 90 : -90);
        }
        zeroAngle = getNavAngle();

    }

    public static PigeonYummy getInstance(){
        if (instance == null){
            instance = new PigeonYummy();
        }
        return instance;
    }

    public double getNavHeading(){
        return getYaw().getValue();
    }

    public double getNavAngle(){
        return getYaw().getValue();
    }

    public double getHeading() {
        return Math.IEEEremainder(-getNavAngle(), 360) - ((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 180 : 0);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetGyro(){
        setYaw(0);
    }

}
