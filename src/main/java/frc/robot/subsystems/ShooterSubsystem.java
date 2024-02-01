package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false; // Controlled by driver

    public CANSparkMax speakerMotorTop;
    public CANSparkMax speakerMotorBottom;

    public CANSparkMax speakerMotorPivot;
    public DutyCycleEncoder pivotEncoder;
    public PIDController pivotPIDController;

    public ShooterSubsystem() {
        speakerMotorTop = new CANSparkMax(Constants.Shooter.shooterMotorTopID, MotorType.kBrushless);
        speakerMotorBottom = new CANSparkMax(Constants.Shooter.shooterMotorBottomID, MotorType.kBrushless);
        speakerMotorPivot = new CANSparkMax(0, MotorType.kBrushless);

        pivotPIDController = new PIDController(0, 0, 0);
        pivotEncoder = new DutyCycleEncoder(0);
    }
}