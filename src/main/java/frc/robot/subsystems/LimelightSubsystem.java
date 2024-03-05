package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    public NetworkTable table;
    public double[] fieldSpacePosition;
    public double[] cameraPoseTargetSpace;
    public double tagDistance;
    public double tx;
    public long tid;
    
    public LimelightSubsystem() {
    }

    @Override
    public void periodic() {
        /* get robots position using the limelight and update it periodically */
        table = NetworkTableInstance.getDefault().getTable("limelight");
        fieldSpacePosition = table.getEntry("botpose").getDoubleArray(new double[6]);
        cameraPoseTargetSpace = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        /* returns distance in inches */
        tagDistance = Math.sqrt(
            Math.pow(cameraPoseTargetSpace[0], 2) +
            Math.pow(cameraPoseTargetSpace[2], 2)
        ) * 39.33;
        tx = table.getEntry("tx").getDouble(0);
        tid = table.getEntry("tid").getInteger(0);
    }
}
