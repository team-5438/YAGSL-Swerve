package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table;
    public double[] fieldSpacePosition;
    public double[] cameraPoseTargetSpace;
    public double tx, ty, tz;
    public long tid;
    
    public LimelightSubsystem() {
        updateValues();
    }

    public void updateValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        fieldSpacePosition = table.getEntry("botpose").getDoubleArray(new double[6]);
        cameraPoseTargetSpace = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        tx = table.getEntry("tx").getDouble(0);
        tid = table.getEntry("tid").getInteger(0);
    }
}