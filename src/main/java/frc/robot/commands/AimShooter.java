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
        double speakerDistance = limelightSubsystem.tagDistanceIn;

		// Only auto align if a tag is in sight and it's the correct tag
        if (speakerDistance != 0 && limelightSubsystem.tagID == Constants.AprilTags.SPEAKER_CENTRAL) {
            double angle = 0.0;
            if(speakerDistance < 260) {
                if (speakerDistance < 36) {
                    angle = 0.135;
                } else if (speakerDistance > 36 && speakerDistance < 48) {
                    angle = -0.0006334 * (speakerDistance - 48) + 0.129;
                } else if (speakerDistance > 48 && speakerDistance < 133.914) {
                    angle = 6.3 / speakerDistance;
                } else if (speakerDistance > 133.914) {
                    angle = -0.000105 * (speakerDistance - 264) + 0.035;
                }
                angle -= 0.005;

				// Rev shooter wheels when a note is loaded
                if (shooterSubsystem.colorSensor.getProximity() > 150) {
					shooterSubsystem.bottomRevMotor.set(-Constants.Shooter.revSpeed);
					shooterSubsystem.topRevMotor.set(Constants.Shooter.revSpeed);
                } else {
                    shooterSubsystem.bottomRevMotor.set(0);
                    shooterSubsystem.topRevMotor.set(0);
                }
            } else {
                angle = 0.14;
            }

			// PID to the calculated angle
            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(Math.abs(shooterSubsystem.pivotEncoder.getPosition()), angle);
			shooterSubsystem.pivotMotor.set(sp * 10);

            if (shooterSubsystem.pivotEncoder.getPosition() < -0.14 && sp > 0) {
                shooterSubsystem.pivotMotor.set(0);
            } else {
				shooterSubsystem.pivotMotor.set(sp * 10);
			}

            if (Math.abs(shooterSubsystem.pivotEncoder.getPosition() - angle) < 0.02) {
                LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 0, 255, 0));
            } else {
                LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 255, 255, 255));
            }
        } else {
			// If a speaker central tag is not in sight rever to manual aiming
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

			// TODO: fix manual pivot feed forward
            // if(shooterSubsystem.pivotEncoder.getPosition() < -0.9) {
                // sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getPosition()), sp, 0.2);
            // }
			
            shooterSubsystem.pivotMotor.set(sp);
        }
        if (shooterSubsystem.colorSensor.getProximity() < 150) {
            shooterSubsystem.bottomRevMotor.set(0);
            shooterSubsystem.topRevMotor.set(0);
        }
    }
}
