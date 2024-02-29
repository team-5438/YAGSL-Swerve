// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.Constants.DriverConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* initalize swerve with our config */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ClimberSubsystem climberSubsystem  = new ClimberSubsystem();

  /* initialize controllers */
  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  private final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...
    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(4));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureBindings() {
    /* zero gyro when pressing Y on xbox controller */
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(
      new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driver, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driver, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    new JoystickButton(operator, PS4Controller.Button.kShare.value).whileTrue(new ClimbCommand(climberSubsystem, 1));
    new JoystickButton(operator, PS4Controller.Button.kOptions.value).whileTrue(new ClimbCommand(climberSubsystem, -1));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
