package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
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
        SmartDashboard.putNumber("LL Dist", distance);
        
        // Only Aim shooter and rev shooter if in
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

                double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(
                    shooterSubsystem.pivotEncoder.getPosition(), angle / 360);

                SmartDashboard.putNumber("Target Angle", (angle / 360));
                SmartDashboard.putNumber("Current Angle", shooterSubsystem.pivotEncoder.getPosition());
                SmartDashboard.putNumber("Speed", sp);

                shooterSubsystem.speakerMotorPivot.set(sp * 10);
                // shooterSubsystem.speakerMotorTop.set(0.1);
                // shooterSubsystem.speakerMotorBottom.set(-0.1);
				//
				// TODO TODO TODO TODO TODO TODO TODO
				// Print Ryan's Formula to shuffleboard and ensure that it alone is working
				// This means get a limelight distance, ENSURE THAT IS ACCURATE. (still need to use pythagoras for final distance, only considering z distance rn)
				// One limelight is good, use bot and ll to give distances to shuffleboard so we can MANUALLY CHECK IF THEY ARE RIGHT
				// If the angles it returns are right we should put them on the bot and then handle EDGE CASES. i.e. no tags, bad input etc
            // }
        } else {
			// MANUAL AIMING / NON SHOOTER MODE controls for the shooter
            double sp = MathUtil.applyDeadband(-operator.getRightY(), Constants.OperatorConstants.RIGHT_Y_DEADBAND);

            // stop the shooter from going into the ground
            if (sp > 0)
                sp = Math.abs((shooterSubsystem.pivotEncoder.getPosition() - 0.257) * sp);
            else if (sp < 0)
                sp = (shooterSubsystem.pivotEncoder.getPosition() > 0.9 ? 0 : shooterSubsystem.pivotEncoder.getPosition())
                    * sp;

			// Safety first <3 (speed clamp)
            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

            shooterSubsystem.speakerMotorPivot.set(sp); 

			// Look into why this sheet dont work
            if(operator.getTriangleButton()) {
                shooterSubsystem.speakerMotorTop.set(0.01);
                // shooterSubsystem.speakerMotorBottom.set(-0.1);
            }
        }
    }
}
