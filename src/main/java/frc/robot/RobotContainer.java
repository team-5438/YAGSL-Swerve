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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimShooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AlignWithSpeaker;
import frc.robot.commands.AmpPreset;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.RevFeedWheels;
import frc.robot.commands.RevShooterWheels;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public final AmpSubsystem ampSubsystem = new AmpSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  public final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);
  double pov;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private final AlignWithSpeaker alignWithSpeaker = new AlignWithSpeaker(limelightSubsystem, drivebase);
  private final AimShooter aimShooter = new AimShooter(shooterSubsystem, limelightSubsystem, operator);
  private final AmpShoot ampShoot = new AmpShoot(ampSubsystem, shooterSubsystem, operator);

  public RobotContainer() {
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(4)
    );

    // Switch for driveFieldOrientedDirectAngleSim if thats needed
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    ampSubsystem.setDefaultCommand(ampShoot);
    shooterSubsystem.setDefaultCommand(aimShooter);
  }

  private void configureBindings() {
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driver, XboxController.Button.kA.value).onTrue(alignWithSpeaker);
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    new JoystickButton(operator, PS4Controller.Button.kTriangle.value).onTrue(new InstantCommand(() -> shooterSubsystem.toggleShooterMode(), shooterSubsystem));
    new JoystickButton(operator, PS4Controller.Button.kL2.value).whileTrue(new RevShooterWheels(shooterSubsystem));
    new JoystickButton(operator, PS4Controller.Button.kSquare.value).onTrue(new SequentialCommandGroup(
      new IntakeCommand(intakeSubsystem, shooterSubsystem),
      new InstantCommand(() -> shooterSubsystem.feedMotor.set(-0.21)),
      new WaitCommand(0.04),
      new InstantCommand(() -> shooterSubsystem.feedMotor.set(0))
    ));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value).onTrue(new InstantCommand(() -> shooterSubsystem.toggleShooterMode()));
    new JoystickButton(operator, PS4Controller.Button.kR2.value).onTrue(new ShootCommand(shooterSubsystem).withTimeout(1));
    new JoystickButton(operator, PS4Controller.Button.kCross.value).onTrue(new AmpPreset(0.25, ampSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void printToDashboard() {
    SmartDashboard.putNumber("tx", limelightSubsystem.tx);
    SmartDashboard.putNumber("ti", limelightSubsystem.tid);
  }
}
