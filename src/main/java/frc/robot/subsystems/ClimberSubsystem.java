package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class ClimberSubsystem extends SubsystemBase{
    public CANSparkMax LClimber;
    public CANSparkMax RClimber;

    public ClimberSubsystem() {
        LClimber = new CANSparkMax(Constants.ClimberConstants.LClimberID, MotorType.kBrushless);
        RClimber = new CANSparkMax(Constants.ClimberConstants.RClimberID, MotorType.kBrushless);
    }
}
