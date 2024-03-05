package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false;

    public CANSparkMax speakerMotorTop;
    public CANSparkMax speakerMotorBottom;
    public CANSparkMax feedMotor;
    public double feedVoltage;
    public ColorSensorV3 colorSensor;

    public CANSparkMax speakerMotorPivot;
    public RelativeEncoder pivotEncoder;
    public PIDController pivotPIDControllerAuto;
    public PIDController pivotPIDControllerManual;

    public ArmFeedforward pivotFeedforward;

    public ShuffleboardTab tab;

    public ShooterSubsystem() {
        speakerMotorTop = new CANSparkMax(Constants.Shooter.shooterMotorTopID, MotorType.kBrushless);
        speakerMotorBottom = new CANSparkMax(Constants.Shooter.shooterMotorBottomID, MotorType.kBrushless);
        speakerMotorPivot = new CANSparkMax(Constants.Shooter.pivotMotorID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.shooterFeedMotorID, MotorType.kBrushless);

        pivotPIDControllerAuto = new PIDController(2, 0, 0.0);
        pivotPIDControllerAuto.enableContinuousInput(0, 1);

        pivotPIDControllerManual = new PIDController(0.1, 0, 0);
        pivotPIDControllerManual.enableContinuousInput(0, 1);

        // pivotEncoder = new SparkAbsoluteEncoder(CANSparkBase.)
        pivotEncoder = speakerMotorPivot.getEncoder();
        // pivotEncoder = new DutyCycleEncoder(Constants.Shooter.pivotEncoderID);
        // pivotEncoder.setPositionOffset(Constants.Shooter.pivotEncoderOffset);

        pivotFeedforward = new ArmFeedforward(0, 0.0, 0.02, 0.05);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        /* Invert motor so we shoot outwards */
        feedMotor.setInverted(true);
    }

    /* we need to invert the encoder angle because it spins opposite of the shooter */
    public double fixEncoderAngle(double angle) {
        return Math.abs(angle);
    }

    public void toggleShooterMode() {
        isAutoRunning = !isAutoRunning;
        System.out.println(isAutoRunning ? "Is in shooter mode" : "Is NOT in shooter mode");
    }
}
