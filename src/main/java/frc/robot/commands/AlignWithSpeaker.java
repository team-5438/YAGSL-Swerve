package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithSpeaker extends Command {
    LimelightSubsystem ll;
    boolean aligned;
    Translation2d zero;
    SwerveSubsystem swerve;
    double rotSpeed;
    double tolerance;
    double offsetTagOffset;
    PIDController rotationPID;

    public AlignWithSpeaker(LimelightSubsystem _limelight, SwerveSubsystem _swerve) {
        ll = _limelight;
        swerve = _swerve;
        tolerance = 2.5;
        zero = new Translation2d(0, 0);
        rotationPID = new PIDController(0.5, 0.0, 0.00);
        this.addRequirements(swerve);
    }

    @Override
    public void execute() {
        rotSpeed = rotationPID.calculate(ll.tx, 0.0) / 6;
        // rotSpeed = ll.tx > 0 ? -0.2 : 0.2;
        swerve.drive(zero, -rotSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.led0.setData(LEDCommand.setStripColor(27, 0, 255, 0));
    }
}