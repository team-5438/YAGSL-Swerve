package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithAmp extends Command {
    boolean aligned;
    SwerveSubsystem swerveSubsystem;
	double alignedTolerance;
    /* TO DO
     * Update the ampPreset for the actual values
     * Interrupt the command if Joystick is moved
     */
    Pose2d ampPreset = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? new Pose2d(new Translation2d(2,2), new Rotation2d(0.1)) : new Pose2d(new Translation2d(2,2), new Rotation2d(0.1)); 

    public AlignWithAmp(SwerveSubsystem swerveSubsystem) {
        this.addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        LEDSubsystem.aquireLock("Align", LEDSubsystem.led0);
        LEDSubsystem.setStrip("Align", LEDSubsystem.led0, LEDCommand.setStripColor(27, 255, 165, 0));
    }

    @Override
    public void execute() {
        swerveSubsystem.driveToPose(ampPreset);
        LEDSubsystem.setStrip("Align", LEDSubsystem.led0, LEDCommand.setStripColor(27, 0, 255, 0));
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.releaseLock("Align", LEDSubsystem.led0);
    }
}
