package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Conversions;

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
        

        if (shooterSubsystem.isAutoRunning) {
            // double[] distanceList = {121.92, 198.12, 259.08, 289.56, 335.28, 365.76, 396.64, 426.72, 670.56};
            // double[] angleList = {0.129, 0.08, 0.063, 0.056, 0.048, 0.0465, 0.045, 0.044, 0.035};

            // double ceilingDistance = distanceList[0];
            // double ceilingAngle = angleList[0];
            // double floorDistance = distanceList[distanceList.length - 1];
            // double floorAngle = angleList[angleList.length - 1];
            // for(int i = 0; i < distanceList.length; i++) {
            //     if(distanceList[i] > floorDistance && distanceList[i] < distance) {
            //         floorDistance = distanceList[i];
            //         floorAngle = angleList[i];
            //         ceilingAngle = angleList[i + 1];
            //         ceilingDistance = distanceList[i + 1];
            //     }
            // }
            // double t = distance - floorDistance / (ceilingDistance - floorDistance);
            // double angle = floorAngle + t * (ceilingDistance - floorDistance);

            // double heightDif = 78 - Constants.Shooter.height;
            // double dif2 = Math.pow(heightDif, 2);
            // double e2 = Math.pow(18, 2);
            // double fesf2 = Math.pow(4.875, 2);
            // double shooterHeight = heightDif/distance;
            // double v = (180/Math.PI) * Math.asin(Math.sin(Math.atan(shooterHeight)+Math.atan(4.875/18))) *
            //         (Math.sqrt((fesf2)+(e2))) / (Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-(2 *
            //         (Math.sqrt((fesf2)+(e2)) * (Math.sqrt((distance*distance)+(dif2))) *
            //         Math.cos(Math.atan(shooterHeight)+Math.atan(4.875/18))))));

            // double min = (180/Math.PI)*Math.atan(shooterHeight);
            // double max = min + v;
                
            // Get average of the two, favoring the top cuz grabiby
            // double angle = (min * 1.2 + max) / 2;

            double angle = 0.0;
            distance = Math.sqrt(Math.pow(distance, 2)-1849);

            /* encoder distance */
            if(distance < 260 && distance != 0 && (limelightSubsystem.tid == Constants.AprilTags.SPEAKER_CENTRAL || limelightSubsystem.tid == Constants.AprilTags.SPEAKER_OFFSET)) {
                if (distance < 36) {
                    angle = 0.14 - 0.005;
                } else if (distance > 36 && distance < 48) {
                    angle = -0.0006334 * (distance - 48) + 0.129;
                } else if (distance > 48 && distance < 133.914) {
                    angle = 6.3 / distance;
                } else if (distance > 133.914) {
                    angle = -0.0000925925926 * (distance - 264) + 0.035;
                }
                angle += 0.005;
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

            if (Math.abs(shooterSubsystem.pivotEncoder.get() - angle) < 0.01) {
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
