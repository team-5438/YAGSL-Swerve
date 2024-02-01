package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix.CTREJNIWrapper;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.utils.Conversions;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false; // Controlled by driver

    private AnalogInput angleEncoder;
    // Speaker Shooter
    public CANSparkMax speakerMotorTop = new CANSparkMax(Constants.Shooter.shooterMotorTopID, MotorType.kBrushless);
    public CANSparkMax speakerMotorBottom = new CANSparkMax(Constants.Shooter.shooterMotorBottomID, MotorType.kBrushless);
    public CANSparkMax speakerMotorPivot = new CANSparkMax(0, MotorType.kBrushless);

    private PIDController pivotPIDController = new PIDController(0, 0, 0);
    private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0); // "Initializes a duty cycle encoder on DIO pins 0" fuck is a DIO pin bro

    // Amp Shooter

    public CANSparkMax ampWheelMotor = new CANSparkMax(Constants.Shooter.ampMotorID, MotorType.kBrushless);
    public CANSparkMax ampRotationMotor = new CANSparkMax(0, MotorType.kBrushless);
    
    private SparkAbsoluteEncoder ampEncoder = ampRotationMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private PIDController ampPIDController = new PIDController(0, 0, 0);

    public void setSpeakerAngle(double desiredAngle)
    {
        double output = pivotPIDController.calculate(pivotEncoder.getAbsolutePosition(), Conversions.degreesToSparkMax(desiredAngle, 0)); // TODO: Get gear ratio
        speakerMotorPivot.set(output);  
    }

    public void setSpeakerMotor(double desiredSpeed)
    {
        speakerMotorTop.set(desiredSpeed);
        speakerMotorBottom.set(desiredSpeed);
    }

    public void setAmpAngle(double desiredAngle)
    {
        double outputAngle = ampPIDController.calculate(ampEncoder.getPosition(), Conversions.degreesToSparkMax(desiredAngle, 0));
        ampRotationMotor.set(outputAngle);
    }

    public void setAmpMotor(double desiredSpeed)
    {
        ampWheelMotor.set(desiredSpeed);
    }  

    public Command startAutoAim()
    {
        return new AutoAimCommand(this, 0); // TODO: Get distance from somewhere
    }

    private Rotation2d getEncoder() {
        return Rotation2d.fromRadians((angleEncoder.getVoltage()
        / RobotController.getVoltage5V()) * 2 * Math.PI);
    }
    
    public Command onInit()
    {
        double absolutePosition = Conversions.degreesToSparkMax( getEncoder().getDegrees() - Constants.Shooter.angleOffset.getDegrees(), Constants.Shooter.pivotMotorID);
        speakerMotorPivot.set(absolutePosition);
        return null;
    }    
}