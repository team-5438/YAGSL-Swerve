package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
    public ShooterSubsystem shooterSubsystem;
    // public LimelightSubsystem limelightSubsystem;
    public PhotonSubsystem photonSubsystem;
    private PhotonTrackedTarget tag;
    public PS4Controller operator;

    public AimShooter(ShooterSubsystem shooterSubsystem, PhotonSubsystem photonSubsystem, PS4Controller operator) {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        // this.limelightSubsystem = limelightSubsystem;
        this.photonSubsystem = photonSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {
        // Shuffleboard.getTab("Shooter").add("Encoder Angle", Math.abs(shooterSubsystem.pivotEncoder.getDistance()));
        double speakerDistance = 0.0;
        // double speakerDistance = limelightSubsystem.tagDistanceIn;

        // get the distance and figure out actual distance using pythagorian formula
        if ((tag = photonSubsystem.getTag(Constants.AprilTags.SPEAKER_CENTRAL)) != null)
            speakerDistance = Units.metersToInches(Math.abs(tag.getBestCameraToTarget().getX() - 1.592));
        else
            speakerDistance = 0;

        if (speakerDistance != 0) {
            double angle = 0.0;
            if(speakerDistance < 260) {
                if (speakerDistance < 36) {
                    angle = 0.14;
                } else if (speakerDistance > 36 && speakerDistance < 48 + 2) {
                    angle = -0.0006334 * (speakerDistance - 48) + 0.129;
                } else if (speakerDistance > 48 && speakerDistance < 133.914 + 2) {
                    angle = (6.35 / speakerDistance);
                } else if (speakerDistance > 133.914 + 2) {
                    angle = -0.000105 * (speakerDistance - 264) + 0.035;
                }
                angle += 0.00;

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
            // Shuffleboard.getTab("Shooter").add("Desired Angle", angle);
            // Shuffleboard.getTab("Shooter").add("Speaker Distance", speakerDistance);
            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(Math.abs(shooterSubsystem.pivotEncoder.getDistance()), angle);
			shooterSubsystem.pivotMotor.set(sp * 10);

            if (shooterSubsystem.pivotEncoder.getDistance() < -0.14 && sp > 0) {
                shooterSubsystem.pivotMotor.set(0);
            } else {
				shooterSubsystem.pivotMotor.set(sp * 10);
			}

            if (Math.abs(shooterSubsystem.pivotEncoder.getDistance() - angle) < 0.02) {
                LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 0, 255, 0));
            } else {
                LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 255, 165, 0));
            }

            if (shooterSubsystem.colorSensor.getProximity() < 150) {
                shooterSubsystem.bottomRevMotor.set(0);
                shooterSubsystem.topRevMotor.set(0);
            }
        } else {
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

			// TODO: fix manual pivot feed forward
            // if(shooterSubsystem.pivotEncoder.getDistance() < -0.9) {
                // sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getDistance()), sp, 0.2);
            // }
			
            shooterSubsystem.pivotMotor.set(sp);
        }
    }
}
