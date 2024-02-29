package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;

public class AmpPreset extends Command {
    AmpSubsystem ampSubsystem;
    double encoderSetPoint;

    public AmpPreset(double encoderSetPoint, AmpSubsystem ampSubsystem) {
        this.addRequirements(ampSubsystem);
        this.ampSubsystem = ampSubsystem;
        this.encoderSetPoint = encoderSetPoint;
    }

    @Override
    public void execute() {
        System.out.println(ampSubsystem.ampPivotEncoder.getPosition());
        double sp = -ampSubsystem.ampPivotPIDController.calculate(ampSubsystem.ampPivotEncoder.getPosition(), encoderSetPoint);
        // sp += ampSubsystem.ampFeedforward.calculate(ampSubsystem.ampPivotEncoder.getPosition(), sp);
        ampSubsystem.ampPivotMotor.set(sp);
    }

    public boolean isFinished() {
        return Math.abs(ampSubsystem.ampPivotEncoder.getPosition() - encoderSetPoint) < 0.02;
    }
}
