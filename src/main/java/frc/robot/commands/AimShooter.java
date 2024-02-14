package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
    public ShooterSubsystem shooterSubsystem;
    public LimelightSubsystem limelightSubsystem;
    public PS4Controller operator;

    public AimShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, PS4Controller operator)
    {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {
        double distance = limelightSubsystem.speakerDistance;
        
        // Only Aim shooter and rev shooter if in
        if(distance <= Constants.Shooter.shooterModeMinDistance * 39.37) {
            LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(27, 252, 215, 0));

            // All of Ryans formula CURRENTLY WE DON'T KNOW INPUTS AND OUTPUTS
            int heightDif = 78 - Constants.Shooter.height;
            double dif2 = Math.pow(heightDif, 2);
            double e2 = Math.pow(18, 2);
            double fesf2 = Math.pow(4.875, 2);
            double v = (180/Math.PI)*Math.asin(Math.sin(Math.atan(heightDif/distance)+Math.atan(4.875/18)))*(Math.sqrt((fesf2)+(e2)))/(Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-2*(Math.sqrt((fesf2)+(e2))*(Math.sqrt((distance*distance)+(dif2))))));
            double min = (180/Math.PI)*Math.atan(heightDif/distance);
            double max = min + v;
            
            double angle = (min*1.2 + max) / 2;

            double sp = shooterSubsystem.pivotPIDControllerAuto.calculate(
                shooterSubsystem.pivotEncoder.getPosition(),
                (angle % 360) / 360
            );
            shooterSubsystem.speakerMotorPivot.set(sp); 
            shooterSubsystem.speakerMotorTop.set(0.1);
            shooterSubsystem.speakerMotorBottom.set(-0.1);
        } else {
            // Handle all manual control for aiming the shooter 
            double shooterSpeed = operator.getRightY();
            double sp;

            // Use PID to get move the shooter while HOPEFULLY slowing when reaching the top and bottom clamps
            if(shooterSpeed < -Constants.OperatorConstants.RIGHT_Y_DEADBAND) {
                sp = shooterSubsystem.pivotPIDControllerManual.calculate(shooterSubsystem.pivotEncoder.getPosition(), Constants.Shooter.topPivotClamp);
            } else if (shooterSpeed > Constants.OperatorConstants.RIGHT_Y_DEADBAND) {
                sp = shooterSubsystem.pivotPIDControllerManual.calculate(shooterSubsystem.pivotEncoder.getPosition(), Constants.Shooter.bottomPivotClamp);
            } else {
                sp = 0;
            }
            sp = MathUtil.clamp(sp, -Constants.Shooter.manualPivotSpeedClamp, Constants.Shooter.manualPivotSpeedClamp);

            shooterSubsystem.speakerMotorPivot.set(sp); 

            if(operator.getTriangleButton()) {
                shooterSubsystem.speakerMotorTop.set(0.1);
                shooterSubsystem.speakerMotorBottom.set(-0.1);
            }

            LEDSubsystem.sponsorStrip1.setData(LEDCommand.setStripColor(27, 0, 0, 0));
        }
    }

    public boolean isFinished() {
        return !shooterSubsystem.isAutoRunning;
    }
}