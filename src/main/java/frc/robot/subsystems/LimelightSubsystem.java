package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    public NetworkTable shooterCameraTable;
    public double[] cameraPoseTargetSpace;
    public double tagDistanceIn;
    public double tagOffsetX;
    public long tagID;

    @Override
    public void periodic() {
        shooterCameraTable = NetworkTableInstance.getDefault().getTable("limelight");
        cameraPoseTargetSpace = shooterCameraTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        tagDistanceIn = Math.sqrt(
            Math.pow(cameraPoseTargetSpace[0], 2) +
            Math.pow(cameraPoseTargetSpace[2], 2)
        ) * 39.33;
        tagOffsetX = shooterCameraTable.getEntry("tx").getDouble(0);
        tagID = shooterCameraTable.getEntry("tid").getInteger(0);
    }
}
