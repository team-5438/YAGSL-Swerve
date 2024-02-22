// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AlignWithSpeaker;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  //private final AmpShooterSubsystem ampShooterSubsystem = new AmpShooterSubsystem();

  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  private final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);

  private final AlignWithSpeaker alignWithSpeaker = new AlignWithSpeaker(limelightSubsystem, drivebase);
  private final AimShooter aimShooter = new AimShooter(shooterSubsystem, limelightSubsystem, operator);
  //private final AmpShoot ampShoot = new AmpShoot(ampShooterSubsystem, 0.22, shooterSubsystem, 0.34);

  public RobotContainer() {
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(4)
    );

    // Switch for driveFieldOrientedDirectAngleSim if thats needed
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    shooterSubsystem.setDefaultCommand(aimShooter);
  }

  private void configureBindings() {
    /* zero gyro when pressing Y on xbox controller */
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driver, XboxController.Button.kA.value).onTrue(alignWithSpeaker);
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    new JoystickButton(operator, PS4Controller.Button.kCircle.value).onTrue(new InstantCommand(() -> shooterSubsystem.toggleShooterMode(), shooterSubsystem));
    //new JoystickButton(operator,PS4Controller.Button.kL1.value).onTrue(ampShoot);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void printToDashboard() {
    SmartDashboard.putNumber("tx", limelightSubsystem.tx);
    SmartDashboard.putNumber("ti", limelightSubsystem.tid);
  }
}
