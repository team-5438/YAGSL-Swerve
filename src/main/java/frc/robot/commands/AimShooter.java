package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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

    public AimShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, PS4Controller operator) {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {        
        double distance = limelightSubsystem.speakerDistance;
        

        if (distance != 0 && (limelightSubsystem.tid == Constants.AprilTags.SPEAKER_CENTRAL || limelightSubsystem.tid == Constants.AprilTags.SPEAKER_OFFSET)) {
            double angle = 0.0;
            /* encoder distance */
            if(distance < 260) {
                if (distance < 36) {
                    angle = 0.14;
                } else if (distance > 36 && distance < 48) {
                    angle = -0.0006334 * (distance - 48) + 0.129;
                } else if (distance > 48 && distance < 133.914) {
                    angle = 6.3 / distance;
                } else if (distance > 133.914) {
                    angle = -0.000105 * (distance - 264) + 0.035;
                }
                angle += 0.0066;
                if (shooterSubsystem.colorSensor.getProximity() > 150) {
                    shooterSubsystem.speakerMotorBottom.set(-Constants.Shooter.shooterRevSpeed);
                    shooterSubsystem.speakerMotorTop.set(Constants.Shooter.shooterRevSpeed);
                } else {
                    shooterSubsystem.speakerMotorBottom.set(0);
                    shooterSubsystem.speakerMotorTop.set(0);
                }
            } else {
                angle = 0.14;
            }

            System.out.println(angle);
            System.out.println("Distance: " + distance);

            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(
                shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getDistance()), angle);

            if (shooterSubsystem.pivotEncoder.getDistance() < -0.14 && sp > 0) {
                shooterSubsystem.speakerMotorPivot.set(0);
            } else
                shooterSubsystem.speakerMotorPivot.set(sp * 10);

            if (Math.abs(shooterSubsystem.pivotEncoder.get() - angle) < 0.02) {
                LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(27, 0, 255, 0));
            } else {
                LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(27, 235, 229, 52));
            }
        } else {
			// MANUAL AIMING / NON SHOOTER MODE controls for the shooter
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            System.out.println("Encoder angle: " + shooterSubsystem.pivotEncoder.getDistance());

            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

            // if(shooterSubsystem.pivotEncoder.getDistance() < -0.9) {
                sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getDistance()), sp, 0.2);
            // }

            if (shooterSubsystem.pivotEncoder.getDistance() < -0.14 && sp > 0) {
                shooterSubsystem.speakerMotorPivot.set(0);
            } else
                shooterSubsystem.speakerMotorPivot.set(sp);
        }
        if (shooterSubsystem.colorSensor.getProximity() < 150) {
            shooterSubsystem.speakerMotorBottom.set(0);
            shooterSubsystem.speakerMotorTop.set(0);
        }
    }
}
