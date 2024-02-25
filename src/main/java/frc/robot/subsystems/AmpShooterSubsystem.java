package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpShooterSubsystem extends SubsystemBase {
    public CANSparkMax ampShootMotor;
    public CANSparkMax ampPivotMotor;
    
    public SparkAbsoluteEncoder ampPivotEncoder;
    public PIDController ampPivotPIDController;

    public AmpShooterSubsystem() {
        ampShootMotor = new CANSparkMax(Constants.Shooter.ampShootMotorID, MotorType.kBrushless);
        ampPivotMotor = new CANSparkMax(Constants.Shooter.ampPivotMotorID, MotorType.kBrushless);
        ampPivotEncoder = ampPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        ampPivotPIDController = new PIDController(0.1, 0, 0);
        ampPivotPIDController.enableContinuousInput(0, 1);

        ampPivotEncoder.setZeroOffset(0.078);
    }
}