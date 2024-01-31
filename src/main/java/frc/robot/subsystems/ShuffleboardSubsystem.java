package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class ShuffleboardSubsystem {
   public double sp;

   public static ShuffleboardSubsystem Instance;

   public ShuffleboardSubsystem()
   {
      if (Instance == null)
         Instance = this;
   }

   // makes a new tab for all the controls
   private ShuffleboardTab tab = Shuffleboard.getTab("controls");
   private ShuffleboardTab test = Shuffleboard.getTab("Test for saagar");


   public GenericEntry speed =
      tab.add("speed divisor", Constants.speedDivisor).withWidget(BuiltInWidgets.kNumberSlider)
         .withProperties(Map.of("min", 1, "max", 3, "blockIncrement", 0.1))
         .withPosition(0, 0)
         .getEntry();
   
   // gets the value of any entry you pass to it 
   public double getEntry(GenericEntry get, double baseValue) {
      return get.getDouble(baseValue);
   }

   // sets the value of any entry you pass to it
   public void setEntry(GenericEntry set, double value) {
      set.setDouble(value);
   }
 }