// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriverConstants;

import frc.robot.commands.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AlignWithSpeaker;

import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private final AlignWithSpeaker alignWithSpeaker = new AlignWithSpeaker(limelightSubsystem, drivebase);
  public final AimShooter aimShooter = new AimShooter(shooterSubsystem, limelightSubsystem);

  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  private final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    /* absolute driving */
    AbsoluteFieldDrive closedAbsoluteDriveAdv = new AbsoluteFieldDrive(drivebase,
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getRightX(), DriverConstants.RIGHT_X_DEADBAND));

    /*
     * Applies deadbands and inverts controls because joysticks
     * are back-right positive while robot
     * controls are front-left positive
     * left stick controls translation
     * right stick controls the desired angle NOT angular rotation
     */
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRightY(),
        () -> driver.getRightY());

    /*
     * Applies deadbands and inverts controls because joysticks
     * are back-right positive while robot
     * controls are front-left positive
     * left stick controls translation
     * right stick controls the angular velocity of the robot
     */
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(4));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2));

    // Switch for driveFieldOrientedDirectAngleSim if thats needed
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* zero gyro when pressing Y on xbox controller */
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driver, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> shooterSubsystem.isAutoRunning = !shooterSubsystem.isAutoRunning ));
    new JoystickButton(driver, XboxController.Button.kA.value).onTrue(alignWithSpeaker);
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void printToDashboard() {
    SmartDashboard.putNumber("tx", limelightSubsystem.tx);
    SmartDashboard.putNumber("ti", limelightSubsystem.tid);
  }
}
