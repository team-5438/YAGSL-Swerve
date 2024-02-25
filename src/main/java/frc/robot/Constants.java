// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  public static class Shooter {
    public static final int shooterMotorTopID = 8;
    public static final int shooterMotorBottomID = 9;
    public static final int ampShootMotorID = 16;
    public static final int ampPivotMotorID = 7;
    public static final double height = 44; // TODO: Get height of shooter to ground
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
    public static final int pivotMotorID = 15;
    public static final int shooterFeedMotorID = 10;
    public static final double shooterRevSpeed = 1;
    public static final double shooterModeMinDistance = 2.25;
    public static final double topPivotClamp = 0.25;
    public static final double bottomPivotClamp = 0;
    public static final double manualPivotSpeedClamp = 0.2;
    public static final int pivotEncoderID = 1;
    public static final double pivotEncoderOffset = 0.3;
  }

  public static class DriverConstants {
    public static final int id = 0;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class OperatorConstants {
    public static final int id = 1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound /* TODO: Update for our robot */
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); /* TODO: Update for our robot */
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag /* TODO: Update for our robot */

  public static final class Auton {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0); /* TODO: Update for our robot */
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01); /* TODO: Update for our robot */

    public static final double MAX_ACCELERATION = 2; /* TODO: Update for our robot */
  }

  public static final class AprilTags {
    public static final long SPEAKER_CENTRAL = 4;
    public static final long SPEAKER_OFFSET = 3;
  }

  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds /* TODO: Update for our robot */
  }
}
