// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  public static class Shooter {

	// Motor IDS
    public static final int topRevMotorID = 8;
    public static final int bottomRevMotorID = 9;
    public static final int ampShootMotorID = 16;
    public static final int pivotMotorID = 15;
    public static final int feedMotorID = 10;
    public static final int ampPivotMotorID = 7;

	// Encoder and sensors
    public static final int pivotEncoderID = 1;

	// Speeds and clamps
    public static final double revSpeed = 0.75;
    public static final double manualPivotSpeedClamp = 0.2;
    public static final double topPivotClamp = 0.25;
    public static final double bottomPivotClamp = 0;

	// Offsets and presets
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
    public static final double pivotEncoderOffset = 0.5117 - 0.03;
    public static final double shooterShootPivotPreset = -0.114;
    public static final double ampShootingPivotPreset = 0.5314;

    public static final double height = 40; // TODO: Get height of shooter to ground
  }

  public static class DriverConstants {
    public static final int id = 0;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ClimberConstants {
    public static final int LClimberID = 18;
    public static final int RClimberID = 17;
  }

  public static class OperatorConstants {
    public static final int id = 1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    public static final double maximumIntakeVoltage = 0;
    public static int intakeMotorID = 4;
    public static int feedMotorID = 10;

  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); /* TODO: Update for our robot */
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class AprilTags {
    public static final long SPEAKER_CENTRAL = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 4 : 7;
    public static final long SPEAKER_OFFSET = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 3 : 8;
  }

  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds /* TODO: Update for our robot */
  }
}
