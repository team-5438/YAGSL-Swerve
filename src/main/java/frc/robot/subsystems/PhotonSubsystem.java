package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    PhotonCamera cam0 = new PhotonCamera("Camera_Module_v1");
    public double distance;
    public int tagID;

    @Override
    public void periodic() {
        PhotonPipelineResult cameraImage = cam0.getLatestResult();

        if  (cameraImage.hasTargets()) {
            PhotonTrackedTarget target = cameraImage.getBestTarget();
            distance = Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) - 1.592);
            tagID = target.getFiducialId();
        }
    }
}