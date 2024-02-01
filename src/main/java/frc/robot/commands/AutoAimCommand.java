package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAimCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private double distance;

    public AutoAimCommand(ShooterSubsystem shooter, double distance)
    {
        this.shooter = shooter;
        this.distance = distance;
    }
    
    @Override
    public void execute()
    {
        int heightDif = 78 - Constants.Shooter.height;
        double dif2 = Math.pow(heightDif, 2);
        double e2 = Math.pow(18, 2);
        double fesf2 = Math.pow(4.875, 2);
        /* 4.875 = vertical distance from upper lip and lower lip of speaker
         * 18 = horizontal distance ^^
         * heightDif = vertical distance from shooter to high bottom lip
         */
        double v = (180/Math.PI)*Math.asin(Math.sin(Math.atan(heightDif/distance)+Math.atan(4.875/18)))*(Math.sqrt((fesf2)+(e2)))/(Math.sqrt((fesf2)+(e2)+(distance*distance)+(dif2)-2*(Math.sqrt((fesf2)+(e2))*(Math.sqrt((distance*distance)+(dif2))))));
        double min = (180/Math.PI)*Math.atan(heightDif/distance);
        double max = min + v;
        
        shooter.setSpeakerAngle((min + max) / 2);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

}