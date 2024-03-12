package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    PhotonCamera cam0 = new PhotonCamera("Camera_Module_v1");
    PhotonPipelineResult cameraImage;
    public double distance;
    public int tagID;

    @Override
    public void periodic() {
        cameraImage = cam0.getLatestResult();

        if (cameraImage.hasTargets()) {
            PhotonTrackedTarget target = cameraImage.getBestTarget();
            distance = target.getBestCameraToTarget().getX();
            tagID = target.getFiducialId();
        }
    }

    public PhotonTrackedTarget getTag(long speakerCentral) {
        for (PhotonTrackedTarget target : cameraImage.getTargets())
            if (target.getFiducialId() == speakerCentral)
                return target;
        return null;
    }
}