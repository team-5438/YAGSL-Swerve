package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false;

    public CANSparkMax topRevMotor;
    public CANSparkMax bottomRevMotor;
    public CANSparkMax feedMotor;
    public CANSparkMax pivotMotor;


    public RelativeEncoder pivotEncoder;
    public PIDController pivotPIDControllerAuto;
    public PIDController pivotPIDControllerManual;

    public ArmFeedforward pivotFeedforward;

    public ColorSensorV3 colorSensor;

    public ShooterSubsystem() {
        topRevMotor = new CANSparkMax(Constants.Shooter.topRevMotorID, MotorType.kBrushless);
        bottomRevMotor = new CANSparkMax(Constants.Shooter.bottomRevMotorID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.Shooter.pivotMotorID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.feedMotorID, MotorType.kBrushless);

        pivotPIDControllerAuto = new PIDController(2, 0, 0.0);
        pivotPIDControllerAuto.enableContinuousInput(0, 1);

        pivotPIDControllerManual = new PIDController(0.1, 0, 0);
        pivotPIDControllerManual.enableContinuousInput(0, 1);

        pivotEncoder = pivotMotor.getEncoder();

        pivotFeedforward = new ArmFeedforward(0, 0.0, 0.02, 0.05);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        /* Invert motor so we shoot outwards */
        feedMotor.setInverted(true);
    }
}
