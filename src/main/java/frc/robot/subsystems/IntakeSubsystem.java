package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);;
    public CANSparkMax feedMotor = new CANSparkMax(Constants.IntakeConstants.feedMotorID, MotorType.kBrushless);

    public void stopMotor(CANSparkMax motor) {
        motor.set(0);
    }

    public double getVoltage(CANSparkMax motor)
    {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }
}