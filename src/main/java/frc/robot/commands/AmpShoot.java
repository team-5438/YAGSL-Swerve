package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
    public AmpShooterSubsystem AmpSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public double desiredAngleAmp;
    public double desiredAngleShooter;

    public AmpShoot (AmpShooterSubsystem AmpSubsystem, double desiredAngleAmp, ShooterSubsystem shooterSubsystem, double desiredAngleShooter) {
        this.AmpSubsystem = AmpSubsystem;
        this.desiredAngleAmp = desiredAngleAmp;  
        this.desiredAngleShooter = desiredAngleShooter;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        double outputAngleShooter = shooterSubsystem.pivotPIDControllerManual.calculate(shooterSubsystem.pivotEncoder.get(), desiredAngleShooter);
        // shooterSubsystem.speakerMotorPivot.set(outputAngleShooter);
        // TODO: check if need to make slower and multiply by 360 if its not in degrese 
        double outputAngleAmp = AmpSubsystem.ampPivotPIDController.calculate(AmpSubsystem.ampPivotEncoder.getPosition(), desiredAngleAmp);
        // AmpSubsystem.ampPivotMotor.set(outputAngleAmp);
        System.out.println(AmpSubsystem.ampPivotEncoder.getPosition());
        // AmpSubsystem.ampShootMotor.set(1);
    }

    @Override
    public boolean isFinished() {
        double roundedAngle = Math.round(AmpSubsystem.ampPivotEncoder.getPosition() * 100);
        roundedAngle /= 100;
        if (roundedAngle + 0.01 >= desiredAngleAmp && roundedAngle - 0.01 <= desiredAngleAmp)
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        AmpSubsystem.ampShootMotor.set(0);
    }
}