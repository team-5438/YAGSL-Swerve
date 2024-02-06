package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Conversions;

public class AimShooter extends Command {
    public ShooterSubsystem shooterSubsystem;
    public LimelightSubsystem limelightSubsystem;

    public AimShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem)
    {
        this.addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void execute() {
        double distance = limelightSubsystem.speakerDistance;
        if(shooterSubsystem.isAutoRunning && distance <= 300.0){
            int heightDif = 78 - Constants.Shooter.height;
            double dif2 = Math.pow(heightDif, 2);
            double e2 = Math.pow(18, 2);
            double fesf2 = Math.pow(4.875, 2);
            double v = (180/Math.PI)*Math.asin(Math.sin(Math.atan(heightDif/distance)+Math.atan(4.875/18)))*(Math.sqrt((fesf2)+(e2)))/(Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-2*(Math.sqrt((fesf2)+(e2))*(Math.sqrt((distance*distance)+(dif2))))));
            double min = (180/Math.PI)*Math.atan(heightDif/distance);
            double max = min + v;
            
            double angle = (min*1.2 + max) / 2;

            double sp = shooterSubsystem.pivotPIDController.calculate(
                shooterSubsystem.pivotEncoder.getPosition(),
                (angle % 360) / 360
            ) * 10;
            shooterSubsystem.speakerMotorPivot.set(sp); 
        } else {
            shooterSubsystem.speakerMotorPivot.set(0);
        }
    }

    public boolean isFinished() {
        return !shooterSubsystem.isAutoRunning;
    }
}