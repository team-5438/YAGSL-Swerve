package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public boolean isAutoRunning = false;

    public CANSparkMax speakerMotorTop;
    public CANSparkMax speakerMotorBottom;
    public CANSparkMax shooterFeed;

    public CANSparkMax speakerMotorPivot;
    public DutyCycleEncoder pivotEncoder;
    public PIDController pivotPIDControllerAuto;
    public PIDController pivotPIDControllerManual;

    public ArmFeedforward pivotFeedforward;

    public ShuffleboardTab tab;

    public ShooterSubsystem() {
        speakerMotorTop = new CANSparkMax(Constants.Shooter.shooterMotorTopID, MotorType.kBrushless);
        speakerMotorBottom = new CANSparkMax(Constants.Shooter.shooterMotorBottomID, MotorType.kBrushless);
        speakerMotorPivot = new CANSparkMax(Constants.Shooter.pivotMotorID, MotorType.kBrushless);
        shooterFeed = new CANSparkMax(Constants.Shooter.shooterFeedMotorID, MotorType.kBrushless);

        pivotPIDControllerAuto = new PIDController(0.1, 0, 0.0);
        pivotPIDControllerAuto.enableContinuousInput(0, 1);

        pivotPIDControllerManual = new PIDController(0.1, 0, 0);
        pivotPIDControllerManual.enableContinuousInput(0, 1);

        pivotEncoder = new DutyCycleEncoder(Constants.Shooter.pivotEncoderID);
        pivotEncoder.setPositionOffset(Constants.Shooter.pivotEncoderOffset);

        pivotFeedforward = new ArmFeedforward(0, 0.0, 0.02, 0.05);

        /* Invert motor so we shoot outwards */
        shooterFeed.setInverted(true);

        /* make bottom speaker motor follow the top but inverted */
        speakerMotorBottom.follow(speakerMotorTop, true);
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
