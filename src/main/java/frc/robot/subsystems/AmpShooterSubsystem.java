package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpShooterSubsystem extends SubsystemBase {
    public CANSparkMax ampWheelMotor;
    public CANSparkMax ampFeedMotor;
    public CANSparkMax ampRotationMotor;
    
    public SparkAbsoluteEncoder ampEncoder;
    public PIDController ampPIDController;

    public AmpShooterSubsystem() {
        ampWheelMotor = new CANSparkMax(Constants.Shooter.ampMotorID, MotorType.kBrushless);
        ampRotationMotor = new CANSparkMax(Constants.Shooter.ampRotationMotorID, MotorType.kBrushless);
        ampFeedMotor = new CANSparkMax(Constants.Shooter.ampFeedMotorID, MotorType.kBrushless); 
        ampEncoder = ampRotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        ampPIDController = new PIDController(0.1, 0, 0);
    }
}