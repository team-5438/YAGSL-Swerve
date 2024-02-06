package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooterSubsystem;
import frc.robot.utils.Conversions;

public class AmpShoot extends Command {
    public AmpShooterSubsystem AmpSubsystem;
    public double desiredAngle;

    public AmpShoot (AmpShooterSubsystem AmpSubsystem, double desiredAngle) {
        this.AmpSubsystem = AmpSubsystem;
        this.desiredAngle = desiredAngle;  
    }

    @Override
    public void execute() {
        // TODO: check if need to make slower and multiply by 360 if its not in degrese 
        double outputAngle = AmpSubsystem.ampPIDController.calculate(AmpSubsystem.ampEncoder.getPosition(), Conversions.degreesToSparkMax(desiredAngle, 25));
        AmpSubsystem.ampRotationMotor.set(outputAngle);
        AmpSubsystem.ampFeedMotor.set(1);
        AmpSubsystem.ampWheelMotor.set(1); 
    }
}