package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
    public ShooterSubsystem shooterSubsystem;
    public LimelightSubsystem limelightSubsystem;
    public PS4Controller operator;
    public AddressableLEDBuffer lastLed;
    public AddressableLEDBuffer led;
    public double shootSpeed;

    public AimShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, PS4Controller operator, double shootSpeed) {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.operator = operator;
        this.shootSpeed = shootSpeed;
    }

    @Override
    public void execute() {        
        double distance = limelightSubsystem.speakerDistance;
        System.out.println("Encoder: " + shooterSubsystem.pivotEncoder.getPosition());

        if (distance != 0 && (limelightSubsystem.tid == Constants.AprilTags.SPEAKER_CENTRAL || limelightSubsystem.tid == Constants.AprilTags.SPEAKER_OFFSET)) {
            double angle = 0.0;
            /* encoder distance */
            if(distance < 260) {
                if (distance < 36) {
                    angle = 0.135;
                } else if (distance > 36 && distance < 48) {
                    angle = -0.0006334 * (distance - 48) + 0.129;
                } else if (distance > 48 && distance < 133.914) {
                    angle = 6.3 / distance;
                } else if (distance > 133.914) {
                    angle = -0.000105 * (distance - 264) + 0.035;
                }
                angle -= 0.005;
                if (shooterSubsystem.colorSensor.getProximity() > 150) {
                    if (DriverStation.isAutonomous()) {
                        shooterSubsystem.speakerMotorBottom.set(-1.0);
                        shooterSubsystem.speakerMotorTop.set(1.0);
                    } else {
                        shooterSubsystem.speakerMotorBottom.set(-Constants.Shooter.shooterRevSpeed);
                        shooterSubsystem.speakerMotorTop.set(Constants.Shooter.shooterRevSpeed);
                    }
                } else {
                    shooterSubsystem.speakerMotorBottom.set(0);
                    shooterSubsystem.speakerMotorTop.set(0);
                }
                System.out.println("Angle: " + angle);
            } else {
                System.out.println("HHHHHHHHHHHHHHHHHIIIIIIIII SAAGAR");
                angle = 0.14;
            }

            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(
                shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getPosition()), angle);
            shooterSubsystem.speakerMotorPivot.set(sp * 10);

            if (Math.abs(shooterSubsystem.pivotEncoder.getPosition() - angle) < 0.02) {
                if (lastLed != (led = LEDCommand.setStripColor(27, 0, 255, 0)))
                    LEDSubsystem.sponsorStrip1.setData(led);
            } else {
                if (lastLed != (led = LEDCommand.setStripColor(27, 235, 299, 52)))
                    LEDSubsystem.sponsorStrip1.setData(led);
            }
        } else {
			// MANUAL AIMING / NON SHOOTER MODE controls for the shooter
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            // System.out.println("Encoder angle: " + shooterSubsystem.pivotEncoder.getPosition());

            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

            // if(shooterSubsystem.pivotEncoder.getPosition() < -0.9) {
                sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getPosition()), sp, 0.2);
            // }
            shooterSubsystem.speakerMotorPivot.set(sp);
        }
        if (shooterSubsystem.colorSensor.getProximity() < 150) {
            shooterSubsystem.speakerMotorBottom.set(0);
            shooterSubsystem.speakerMotorTop.set(0);
        }
    }
}
