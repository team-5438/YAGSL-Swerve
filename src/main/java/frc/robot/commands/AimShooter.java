package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Conversions;

public class AimShooter extends Command {
    private ShooterSubsystem shooterSubystem;
    private LimelightSubsystem limelightSubsystem;

    public AimShooter(ShooterSubsystem _shooterSubsystem, LimelightSubsystem _limelightSubsystem)
    {
        shooterSubsystem = _shooterSubsystem;
        limelightSubsystem = _limelightSubsystem;
    }
    
    @Override
    public void execute()
    {
        double distance = limelightSubsystem.getSpeakerDistance();
        int heightDif = 78 - Constants.Shooter.height;
        double dif2 = Math.pow(heightDif, 2);
        double e2 = Math.pow(18, 2);
        double fesf2 = Math.pow(4.875, 2);
        double v = (180/Math.PI)*Math.asin(Math.sin(Math.atan(heightDif/distance)+Math.atan(4.875/18)))*(Math.sqrt((fesf2)+(e2)))/(Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-2*(Math.sqrt((fesf2)+(e2))*(Math.sqrt((distance*distance)+(dif2))))));
        double min = (180/Math.PI)*Math.atan(heightDif/distance);
        double max = min + v;
        
        double angle = (min + max) / 2;
        shooterSubystem.speakerMotorPivot.set(
            shooterSubystem.pivotPIDController.calculate(
                    shooterSubystem.pivotEncoder.getAbsolutePosition(),
                    Conversions.degreesToSparkMax(angle, 0)
                )
            );  
    }
}