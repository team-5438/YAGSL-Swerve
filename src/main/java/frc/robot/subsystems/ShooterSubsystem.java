package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false;

    public CANSparkMax speakerMotorTop;
    public CANSparkMax speakerMotorBottom;

    public CANSparkMax speakerMotorPivot;
    public SparkAbsoluteEncoder pivotEncoder;
    public PIDController pivotPIDController;

    public ShuffleboardTab tab;

    public ShooterSubsystem() {
        // speakerMotorTop = new CANSparkMax(Constants.Shooter.shooterMotorTopID,
        // MotorType.kBrushless);
        // speakerMotorBottom = new CANSparkMax(Constants.Shooter.shooterMotorBottomID,
        // MotorType.kBrushless);
        speakerMotorPivot = new CANSparkMax(Constants.Shooter.pivotMotorID, MotorType.kBrushless);

        pivotPIDController = new PIDController(0.1, 0, 0);
        pivotPIDController.enableContinuousInput(0, 1);
        pivotPIDController.setTolerance(0.01);
        pivotEncoder = speakerMotorPivot.getAbsoluteEncoder(Type.kDutyCycle);

        tab = Shuffleboard.getTab("ShooterSubsystem");
    }
}