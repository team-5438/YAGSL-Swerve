package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpPivot extends Command {
    public AmpSubsystem ampSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public double desiredAngleAmp;
    public double desiredAngleShooter;
    public PS4Controller operator;

    public AmpPivot (AmpSubsystem ampSubsystem, ShooterSubsystem shooterSubsystem, PS4Controller operator) {
        this.addRequirements(ampSubsystem);
        this.ampSubsystem = ampSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {
        double sp = operator.getLeftY() / 5;

        // System.out.println("Amp shooter: " + ampSubsystem.ampPivotEncoder.getPosition());
        sp -= ampSubsystem.ampFeedforward.calculate(ampSubsystem.ampPivotEncoder.getPosition(), sp);
        ampSubsystem.ampPivotMotor.set(sp);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}