package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command{
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public int[][] green = {{0, 255, 0, 100}, {0, 0, 0, 100},{0, 255, 0, 100},{0, 0, 0, 100},{0, 255, 0, 100}, {0, 0, 0, 0}};

    public IntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        LEDSubsystem.aquireLock("Intake", LEDSubsystem.led0);
        LEDSubsystem.setStrip("Intake", LEDSubsystem.led0, LEDCommand.setStripColor(27, 255, 200, 0));
    }

    @Override
    public void execute() {
        shooterSubsystem.feedMotor.set(0.1);
        intakeSubsystem.intakeMotor.set(1);
        System.out.println("Prox: " + shooterSubsystem.colorSensor.getProximity());
    }

    @Override
    public boolean isFinished() {
        if (shooterSubsystem.colorSensor.getProximity() > 150) {
            /* we've detected the note! now end the command */
            new FlashLEDS("Intake", LEDSubsystem.led0, green);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        LEDSubsystem.releaseLock("Intake", LEDSubsystem.led0);
        shooterSubsystem.feedMotor.set(0);
        intakeSubsystem.intakeMotor.set(0);
    }
}
