package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithSpeaker extends Command {
    LimelightSubsystem limelightSubsystem;
    boolean aligned;
    Translation2d translationZero;
    SwerveSubsystem swerveSubsystem;
	double alignedTolerance;
    double rotSpeed;
    double offsetTagOffset;
    PIDController rotationPID;

    public AlignWithSpeaker(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.addRequirements(swerveSubsystem);
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        translationZero = new Translation2d(0, 0);
        rotationPID = new PIDController(0.5, 0.0, 0.0);
		alignedTolerance = 2.5;
    }

    @Override
    public void execute() {
        rotSpeed = rotationPID.calculate(limelightSubsystem.tagOffsetX, 0.0) / 6;
        swerveSubsystem.drive(translationZero, -rotSpeed, false);
		
		// Set LEDs to orange while ALIGNING, green when ALIGNED
		if(Math.abs(limelightSubsystem.tagOffsetX) < alignedTolerance) {
			LEDSubsystem.led0.setData(LEDCommand.setStripColor(27, 0, 255, 0));
		} else {
			LEDSubsystem.led0.setData(LEDCommand.setStripColor(27, 255, 165, 0));
		}
    }
}
