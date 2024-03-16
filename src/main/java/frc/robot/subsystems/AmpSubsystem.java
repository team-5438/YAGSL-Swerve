package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase {
    public CANSparkMax ampShootMotor;
    public CANSparkMax ampPivotMotor;
    
    public static SparkAbsoluteEncoder ampPivotEncoder;
    public PIDController ampPivotPIDController;

    public ArmFeedforward ampFeedforward;

    public AmpSubsystem() {
        ampShootMotor = new CANSparkMax(Constants.Shooter.ampShootMotorID, MotorType.kBrushless);
        ampPivotMotor = new CANSparkMax(Constants.Shooter.ampPivotMotorID, MotorType.kBrushless);
        ampPivotEncoder = ampPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        ampPivotPIDController = new PIDController(0.8, 0, 0);
        ampPivotPIDController.enableContinuousInput(0, 1);

        ampFeedforward = new ArmFeedforward(0.0, 0.0, 1, 0.0);
        ampPivotEncoder.setZeroOffset(0.85);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AmpEncoder", ampPivotEncoder.getPosition());
    }
}