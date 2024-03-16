package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false;
    public static boolean isRevved = false;

    public CANSparkMax topRevMotor;
    public CANSparkMax bottomRevMotor;
    public CANSparkMax feedMotor;
    public CANSparkMax pivotMotor;

    public DutyCycleEncoder pivotEncoder;
    public RelativeEncoder topEncoder;
    public RelativeEncoder bottomEncoder;
    public PIDController pivotPIDControllerAuto;
    public PIDController pivotPIDControllerManual;
    public PIDController pivotPIDControllerLimit;

    public ArmFeedforward pivotFeedforward;

    public double desiredVelocity = 300;

    public ColorSensorV3 colorSensor;

    public ShooterSubsystem() {
        topRevMotor = new CANSparkMax(Constants.Shooter.topRevMotorID, MotorType.kBrushless);
        bottomRevMotor = new CANSparkMax(Constants.Shooter.bottomRevMotorID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.Shooter.pivotMotorID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.feedMotorID, MotorType.kBrushless);

        pivotPIDControllerAuto = new PIDController(0.6, 0, 0);
        pivotPIDControllerAuto.enableContinuousInput(0, 1);

        pivotPIDControllerLimit = new PIDController(2, 0, 0);
        pivotPIDControllerLimit.enableContinuousInput(0, 1);

        pivotPIDControllerManual = new PIDController(0.1, 0, 0);
        pivotPIDControllerManual.enableContinuousInput(0, 1);

        // pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder = new DutyCycleEncoder(Constants.Shooter.pivotEncoderID);
        pivotEncoder.setPositionOffset(Constants.Shooter.pivotEncoderOffset);
        topEncoder = topRevMotor.getEncoder();
        bottomEncoder = bottomRevMotor.getEncoder();

        pivotFeedforward = new ArmFeedforward(0, 0.0, 0.02, 0.05);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        /* Invert motor so we shoot outwards */
        feedMotor.setInverted(true);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder", pivotEncoder.getDistance());
        if(topEncoder.getVelocity() > desiredVelocity && bottomEncoder.getVelocity() > desiredVelocity){
            isRevved = true;
        }
        else{
            isRevved = false;
        }
    }
}
