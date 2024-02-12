package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooterSubsystem;

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
        double outputAngle = AmpSubsystem.ampPivotPIDController.calculate(AmpSubsystem.ampPivotEncoder.getPosition(), desiredAngle);
        AmpSubsystem.ampPivotMotor.set(outputAngle);
        //System.out.println(AmpSubsystem.ampPivotEncoder.getPosition());
        //AmpSubsystem.ampFeedMotor.set(1);
        //AmpSubsystem.ampShootMotor.set(1);
    }

    @Override
    public boolean isFinished() {
        double roundedAngle = Math.round(AmpSubsystem.ampPivotEncoder.getPosition() * 100);
        roundedAngle /= 100;
        System.out.println(roundedAngle);
        if (roundedAngle + 0.01 >= desiredAngle && roundedAngle - 0.01 <= desiredAngle)
            return true;
        return false;
    }
}