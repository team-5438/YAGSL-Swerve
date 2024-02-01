// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AmpShooterSubsystem extends SubsystemBase {
//     public CANSparkMax ampWheelMotor;
//     public CANSparkMax ampRotationMotor;
    
//     private SparkAbsoluteEncoder ampEncoder;
//     private PIDController ampPIDController;

//     public AmpShooterSubsystem() {
//         ampWheelMotor = new CANSparkMax(Constants.Shooter.ampMotorID, MotorType.kBrushless);
//         ampRotationMotor = new CANSparkMax(0, MotorType.kBrushless);
//         ampEncoder = ampRotationMotor.getAbsoluteEncoder(Type.kDutyCycle);
//         ampPIDController = new PIDController(0, 0, 0);
//     }

//     public void setAmpAngle(double desiredAngle)
//     {
//         double outputAngle = ampPIDController.calculate(ampEncoder.getPosition(), Conversions.degreesToSparkMax(desiredAngle, 0));
//         ampRotationMotor.set(outputAngle);
//     }

//     public void setAmpMotor(double desiredSpeed)
//     {
//         ampWheelMotor.set(desiredSpeed);
//     }  
// }