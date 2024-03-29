package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithSpeaker extends Command {
    PhotonSubsystem photonSubsystem;
    boolean aligned;
    Translation2d translationZero;
    SwerveSubsystem swerveSubsystem;
	double alignedTolerance;
    double rotSpeed;
    double offsetTagOffset;
    PIDController rotationPID;
    PhotonTrackedTarget tag;

    public AlignWithSpeaker(PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem) {
        this.addRequirements(swerveSubsystem);
        this.photonSubsystem = photonSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        translationZero = new Translation2d(0, 0);
        rotationPID = new PIDController(0.5, 0.0, 0.0);
		alignedTolerance = 7.5; // degrese
    }

    @Override
    public void initialize() {
        LEDSubsystem.aquireLock("Align", LEDSubsystem.led0);
        LEDSubsystem.setStrip("Align", LEDSubsystem.led0, LEDCommand.setStripColor(27, 255, 165, 0));
    }

    @Override
    public void execute() {
        tag = photonSubsystem.getTag(Constants.AprilTags.SPEAKER_CENTRAL);
        if (tag != null)
            rotSpeed = rotationPID.calculate(tag.getYaw(), 0.0) / 6;
        else
            return;
        swerveSubsystem.drive(translationZero, -rotSpeed, false);
		
		// Set LEDs to orange while ALIGNING, green when ALIGNED
		if (Math.abs(tag.getYaw()) < alignedTolerance) {
            swerveSubsystem.aligned = true;
		} else {
            swerveSubsystem.aligned = false;
		}
        // System.out.println("Yaw: " + tag.getYaw());
        // System.out.println(swerveSubsystem.aligned);
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.releaseLock("Align", LEDSubsystem.led0);
    }
}
