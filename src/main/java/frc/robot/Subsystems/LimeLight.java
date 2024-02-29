package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight {

    private NetworkTable tableEntry;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private double[] botpose;

    public LimeLight(){
        tableEntry = NetworkTableInstance.getDefault().getTable("limelight");
//        this.tx = table.getEntry("tx");
//        this.ty = table.getEntry("ty");
//        this.ta = table.getEntry("ta");
//        this.tv = table.getEntry("tv");
//        this.botpose = getBOTPOSE();
    }
    /* public double[] readValues(){
         return new double[]{getTX(), getTY(), getTA(), getTV()};
     }*/
    public double getTV(){
        return tableEntry.getEntry("tv").getDouble(0);
    }

    public double getTX(){
        return tableEntry.getEntry("tx").getDouble(0);
    }

    public double getTY(){
        return tableEntry.getEntry("ty").getDouble(0);
    }

    public double getTA(){
        return tableEntry.getEntry("ta").getDouble(0);
    }

    public double getTL(){
        return tableEntry.getEntry("tl").getDouble(0);
    }

    public double getCL(){
        return tableEntry.getEntry("cl").getDouble(0);
    }

    public double getTSHORT(){
        return tableEntry.getEntry("tshort").getDouble(0);
    }

    public double getTLONG(){
        return tableEntry.getEntry("tlong").getDouble(0);
    }

    public double getTHOR(){
        return tableEntry.getEntry("tlong").getDouble(0);
    }

    public double getTVERT(){
        return tableEntry.getEntry("tvert").getDouble(0);
    }

    public double getGETPIPE(){
        return tableEntry.getEntry("getpipe").getDouble(0);
    }

    public String getJASON(){
        return tableEntry.getEntry("jason").getString("");
    }

    public String getTCLASS(){
        return tableEntry.getEntry("tclass").getString("");
    }

    public double getTC(){
        return tableEntry.getEntry("tc").getDouble(0);
    }

    public double[] getBOTPOSE(){
        return tableEntry.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double[] getBOTPOSE_WPIBLUE(){
        return tableEntry.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public double[] getBOTPOSE_WPIRED(){
        return tableEntry.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    public double[] getCAMERAPOSE_TARGETSPACE(){
        return tableEntry.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    public double[] getTARGETPOSE_ROBOTSPACE(){
        return tableEntry.getEntry("targetpsoe_robotspace").getDoubleArray(new double[6]);
    }

    public double[] getTARGETPOSE_CAMERASPACE(){
        return tableEntry.getEntry("targetpsoe_cameraspace").getDoubleArray(new double[6]);
    }

    public double[] getBOTPOSE_TARGETSPAVE(){
        return tableEntry.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    public double[] getCAMERAPOSE_ROBOTSPACE(){
        return tableEntry.getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
    }
}
