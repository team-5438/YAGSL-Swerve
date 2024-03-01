// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.AmpPivot;
import frc.robot.commands.AmpPreset;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.RevShooterWheels;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public final AmpSubsystem ampSubsystem = new AmpSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem  = new ClimberSubsystem();

  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  public final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);
  double pov, speedMod = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private final AlignWithSpeaker alignWithSpeaker = new AlignWithSpeaker(limelightSubsystem, drivebase);
  private final AimShooter aimShooter = new AimShooter(shooterSubsystem, limelightSubsystem, operator);
  private final AmpPivot ampPivot = new AmpPivot(ampSubsystem, shooterSubsystem, operator);
  private final AmpShoot ampShoot = new AmpShoot(ampSubsystem, shooterSubsystem);

  public RobotContainer() {

    // Named Commands must be the first code called in RobotContainer
    NamedCommands.registerCommand("AutoAim", new InstantCommand(() -> System.out.println("Shoot")));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> System.out.println("Intaking")));
    NamedCommands.registerCommand("RevIntake", new InstantCommand(() -> System.out.println("Revving Intake Wheels")));
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> System.out.println("Shooting")));

    // The robot's subsystems and commands are defined here...
    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1),
        () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1),
        () -> driver.getRawAxis(4) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1)
    );

    // Switch for driveFieldOrientedDirectAngleSim if thats needed
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    ampSubsystem.setDefaultCommand(ampPivot);
    shooterSubsystem.setDefaultCommand(aimShooter);
  }

  private void configureBindings() {
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driver, XboxController.Button.kA.value).onTrue(alignWithSpeaker);
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(drivebase::addFakeVisionReading));

    new JoystickButton(operator, PS4Controller.Button.kL2.value).whileTrue(new RevShooterWheels(shooterSubsystem));
    new JoystickButton(operator, PS4Controller.Button.kSquare.value).onTrue(new SequentialCommandGroup(
        new IntakeCommand(intakeSubsystem, shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.feedMotor.set(-0.21)),
        new WaitCommand(0.04),
        new InstantCommand(() -> shooterSubsystem.feedMotor.set(0))
    ));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value).onTrue(new InstantCommand(() -> shooterSubsystem.toggleShooterMode(), shooterSubsystem));
    new JoystickButton(operator, PS4Controller.Button.kR2.value).onTrue(new ShootCommand(shooterSubsystem).withTimeout(1));
    new JoystickButton(operator, PS4Controller.Button.kCross.value).onTrue(new SequentialCommandGroup(
        new AmpPreset(0.25, ampSubsystem, shooterSubsystem, true),
        new AmpPreset(0.25, ampSubsystem, shooterSubsystem, false).withTimeout(3)
    ));
    new JoystickButton(operator, PS4Controller.Button.kTriangle.value).whileTrue(ampShoot);

    new JoystickButton(operator, PS4Controller.Button.kOptions.value).whileTrue(new ClimbCommand(climberSubsystem, 0.5));
    new JoystickButton(operator, PS4Controller.Button.kShare.value).whileTrue(new ClimbCommand(climberSubsystem, -0.5));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void printToDashboard() {
    SmartDashboard.putNumber("tx", limelightSubsystem.tx);
    SmartDashboard.putNumber("ti", limelightSubsystem.tid);
  }
}
