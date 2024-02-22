package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        
        // Only aim shooter and rev shooter motors if withing the minimum distance
        if (shooterSubsystem.isAutoRunning && distance > 0) {
			// AUTOMATIC / IN SHOOTER MODE logic or aiming the shooter
            // if(distance <= Constants.Shooter.shooterModeMinDistance * 39.37) {

				
                // All of Ryans formula CURRENTLY WE DON'T KNOW INPUTS AND OUTPUTS
                int heightDif = 78 - Constants.Shooter.height;
                double dif2 = Math.pow(heightDif, 2);
                double e2 = Math.pow(18, 2);
                double fesf2 = Math.pow(4.875, 2);
                double v = (180/Math.PI)*Math.asin(Math.sin(Math.atan(heightDif/distance)+Math.atan(4.875/18)))*(Math.sqrt((fesf2)+(e2)))/(Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-2*(Math.sqrt((fesf2)+(e2))*(Math.sqrt((distance*distance)+(dif2))))));

				// Minimum and maximum angles we can fire at
                double min = (180/Math.PI)*Math.atan(heightDif/distance);
                double max = min + v;
				
				// Get the average of the two, favoring the top cuz grabiby
                double angle = (min * 1.2 + max) / 2;

                if(Math.abs(shooterSubsystem.pivotEncoder.getAbsolutePosition() - angle) < 0.01) {
                    //If robot is aimed properly and ready to shoot
                    LEDCommand.setStripColor(500, 0, 255, 0);
                } else {
                    //If the robot is in shooter mode turn Yellow
                    LEDCommand.setStripColor(500, 235, 229, 52);
                }

                double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(
                    shooterSubsystem.pivotEncoder.getAbsolutePosition(), angle / 360);

                // shooterSubsystem.speakerMotorPivot.set(sp * 10);
                // shooterSubsystem.speakerMotorTop.set(0.1);
                // shooterSubsystem.speakerMotorBottom.set(-0.1);
            // }
        } else {
			// MANUAL AIMING / NON SHOOTER MODE controls for the shooter
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            System.out.println("Encoder angle: " + shooterSubsystem.pivotEncoder.getDistance());

            // stop the shooter from going into the ground
            // if (sp > 0)
            //     sp = Math.abs((shooterSubsystem.pivotEncoder.getAbsolutePosition() - 0.257) * sp);
            // else if (sp < 0)
            //     sp = (shooterSubsystem.pivotEncoder.getAbsolutePosition() > 0.9 ? 0 : shooterSubsystem.pivotEncoder.getAbsolutePosition()) * sp;

			// Safety first <3 (speed clamp)
            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

            // sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.pivotEncoder.getDistance(), sp);
            if(shooterSubsystem.pivotEncoder.getDistance() > -0.13) {
                sp += shooterSubsystem.pivotFeedforward.calculate(shooterSubsystem.pivotEncoder.getDistance(), distance, 0.2);
            }

            shooterSubsystem.speakerMotorPivot.set(sp); 

            if(operator.getTriangleButton()) {
                // shooterSubsystem.speakerMotorTop.set(0.01);
                // shooterSubsystem.speakerMotorBottom.set(-0.1);
            }
        }
    }
}
