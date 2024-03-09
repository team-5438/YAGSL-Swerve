package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
    PhotonCamera cam0 = new PhotonCamera("Camera_Module_v1");
    public double distance;
    public int tagID;

    @Override
    public void periodic() {
        var cameraImage = cam0.getLatestResult();

        if  (cameraImage.hasTargets()) {
            PhotonTrackedTarget target = cameraImage.getBestTarget();
            distance = Math.sqrt(Math.pow(target.getBestCameraToTarget().getX(), 2) - 1.592);
            tagID = target.getFiducialId();
        }
    }
}