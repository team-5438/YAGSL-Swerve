package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        rotationPID = new PIDController(0.35, 0.0, 0.01);
    }

    @Override
    public void initialize() {
        aligned = false;
    }

    @Override
    public void execute() {
        System.out.println(Math.abs(ll.tx + (ll.tid == Constants.AprilTags.SPEAKER_OFFSET ? 13.3 : 0)));
        if(Math.abs(ll.tx + (ll.tid == Constants.AprilTags.SPEAKER_OFFSET ? 13.3 : 0)) < tolerance) {
            aligned = true;
        }
        rotSpeed = rotationPID.calculate(ll.tx + (ll.tid == Constants.AprilTags.SPEAKER_OFFSET ? 13.3 : 0), 0.0) / 6;
        swerve.drive(zero, -rotSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return aligned; 
    }
}