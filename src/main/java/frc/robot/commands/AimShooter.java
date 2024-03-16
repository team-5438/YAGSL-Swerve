package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimShooter extends Command {
    public ShooterSubsystem shooterSubsystem;
    // public LimelightSubsystem limelightSubsystem;
    public PhotonSubsystem photonSubsystem;
    public SwerveSubsystem swerveSubsystem;
    private PhotonTrackedTarget tag;
    public PS4Controller operator;

    public AimShooter(ShooterSubsystem shooterSubsystem, PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem, PS4Controller operator) {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        // this.limelightSubsystem = limelightSubsystem;
        this.photonSubsystem = photonSubsystem;
        this.operator = operator;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
        double speakerDistance = 0.0;

        // get the distance and figure out actual distance using pythagorian formula
        if ((tag = photonSubsystem.getTag(Constants.AprilTags.SPEAKER_CENTRAL)) != null)
            speakerDistance = Units.metersToInches(Math.sqrt(Math.pow(tag.getBestCameraToTarget().getX(), 2) - 1.592));
        else
            speakerDistance = 0;

        /* amp preset / software limit */
        if (AmpSubsystem.ampPivotEncoder.getPosition() > 0.2 && shooterSubsystem.pivotEncoder.getDistance() < 0.082) {
            shooterSubsystem.pivotMotor.set(shooterSubsystem.pivotPIDControllerLimit.calculate(Math.abs(shooterSubsystem.pivotEncoder.getDistance()), 0.082));
            /* returns early just incase */
            return;
        }
        if (speakerDistance != 0) {
            double angle = 0.0;
            if(speakerDistance < 200) {
                LEDSubsystem.aquireLock("Shooter", LEDSubsystem.led0);
                if (speakerDistance < 36) {
                    angle = 0.12;
                } else if (speakerDistance > 36 && speakerDistance < 48 ) {
                    angle = -0.0005734 * (speakerDistance - 48) + 0.129;
                } else if (speakerDistance > 48 && speakerDistance < 133.914 ) {
                    angle = (6.3 / speakerDistance );
                } else if (speakerDistance > 133.914 ) {
                    angle = -0.000105 * (speakerDistance - 264) + 0.035;
                }
                angle += 0.005;

                SmartDashboard.putNumber("Desired Shooter Encoder", angle);
				// Rev shooter wheels when a note is loaded
                if (shooterSubsystem.colorSensor.getProximity() > 150) {
					shooterSubsystem.bottomRevMotor.set(-Constants.Shooter.revSpeed);
					shooterSubsystem.topRevMotor.set(Constants.Shooter.revSpeed);
                } else {
                    shooterSubsystem.bottomRevMotor.set(0);
                    shooterSubsystem.topRevMotor.set(0);
                }
            } else {
                LEDSubsystem.releaseLock("Shooter", LEDSubsystem.led0);
                angle = 0.12;
            }

			// PID to the calculated angle
            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(Math.abs(shooterSubsystem.pivotEncoder.getDistance()), angle);
			shooterSubsystem.pivotMotor.set(sp * 10);

            if (shooterSubsystem.pivotEncoder.getDistance() < -0.14 && sp > 0) {
                shooterSubsystem.pivotMotor.set(0);
            } else {
				shooterSubsystem.pivotMotor.set(sp * 10);
			}

            if (Math.abs(Math.abs(shooterSubsystem.pivotEncoder.getDistance()) - angle) < 0.1) {
                System.out.println("Shooter aligned");
                if(swerveSubsystem.aligned)
                    LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 0, 255, 0));
            } else if (speakerDistance <= 200) {
                LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 255, 165, 0));
            }

            if (shooterSubsystem.colorSensor.getProximity() < 150) {
                shooterSubsystem.bottomRevMotor.set(0);
                shooterSubsystem.topRevMotor.set(0);
            }
        } else {
            LEDSubsystem.setStrip("Shooter", LEDSubsystem.led0, LEDCommand.setStripColor(LEDSubsystem.led0Buffer.getLength(), 1, 1, 1));
            LEDSubsystem.releaseLock("Shooter", LEDSubsystem.led0);
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
