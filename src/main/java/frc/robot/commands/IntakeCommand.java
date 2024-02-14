package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    public IntakeSubsystem intakeSubsystem;
    public int[][] yellow = {{252, 227, 3, 500},{0, 0, 0, 500},{0, 227, 0, 500},{0, 0, 0, 500}};

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        double voltage = intakeSubsystem.getVoltage(intakeSubsystem.intakeMotor);
        if (voltage < Constants.IntakeConstants.maximumIntakeVoltage) {
            intakeSubsystem.intakeMotor.set(1);
            intakeSubsystem.feedMotor.set(1);
            new LEDCommand(yellow).schedule();
        }
        else {
            intakeSubsystem.stopMotor(intakeSubsystem.intakeMotor);
            intakeSubsystem.stopMotor(intakeSubsystem.feedMotor);
        }
    }
}
