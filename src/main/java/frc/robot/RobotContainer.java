// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AlignWithSpeaker;
import frc.robot.commands.AmpPivot;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.RevShooterWheels;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
  public final AmpSubsystem ampSubsystem = new AmpSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ClimberSubsystem climberSubsystem  = new ClimberSubsystem();

  private final XboxController driver = new XboxController(Constants.DriverConstants.id);
  public final PS4Controller operator = new PS4Controller(Constants.OperatorConstants.id);

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  private final AlignWithSpeaker alignWithSpeaker = new AlignWithSpeaker(limelightSubsystem, swerveSubsystem);
  private final AimShooter aimShooter = new AimShooter(shooterSubsystem, photonSubsystem, operator);
  private final AmpPivot ampPivot = new AmpPivot(ampSubsystem, shooterSubsystem, operator);

  public RobotContainer() {
    // Named Commands must be the first code called in the ROBOTCONTAINER CONSTRUCTOR
    NamedCommands.registerCommand("Example", new InstantCommand(() -> System.out.println("Example Command")));
    NamedCommands.registerCommand("Shoot", new ShootCommand(shooterSubsystem, intakeSubsystem).withTimeout(1));
    NamedCommands.registerCommand("Intake", new RevShooterWheels(shooterSubsystem, 1.0).andThen(
      new SequentialCommandGroup(
        new IntakeCommand(intakeSubsystem, shooterSubsystem).withTimeout(5),
        new InstantCommand(() -> shooterSubsystem.feedMotor.set(-0.21)),
        new WaitCommand(0.04),
        new InstantCommand(() -> shooterSubsystem.feedMotor.set(0))
      )
    ));

    // The robot's subsystems and commands are defined here...
    configureBindings();

    DoubleSupplier translationX = () -> MathUtil.applyDeadband(driver.getLeftY(), DriverConstants.LEFT_Y_DEADBAND) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1);
    DoubleSupplier translationY = () -> MathUtil.applyDeadband(driver.getLeftX(), DriverConstants.LEFT_X_DEADBAND) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1);
    DoubleSupplier angularRotationX = () -> driver.getRawAxis(4) / (driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) + 1);
    BooleanSupplier robotOriented = () -> !driver.getLeftBumper();

    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveCommand(translationX, translationY, angularRotationX, robotOriented);
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    ampSubsystem.setDefaultCommand(ampPivot);
    shooterSubsystem.setDefaultCommand(aimShooter);
  }

  private void configureBindings() {
    new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(alignWithSpeaker);
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(swerveSubsystem::addFakeVisionReading));

    RevShooterWheels shootFast = new RevShooterWheels(shooterSubsystem, 0.75);
    ShootCommand feed = new ShootCommand(shooterSubsystem, intakeSubsystem);
    new JoystickButton(operator, PS4Controller.Button.kSquare.value).onTrue(new SequentialCommandGroup(
      new IntakeCommand(intakeSubsystem, shooterSubsystem).withTimeout(5),
      new InstantCommand(() -> shooterSubsystem.feedMotor.set(-0.21)),
      new InstantCommand(() -> shooterSubsystem.feedMotor.set(0))
    ));
    new JoystickButton(operator, PS4Controller.Button.kR2.value).onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> shootFast.schedule()),
      // new WaitCommand(3),
      new InstantCommand(() -> feed.schedule()),
      new WaitCommand(0.5),
      new InstantCommand(() -> shootFast.cancel()),
      new InstantCommand(() -> feed.cancel())
    ));
    new JoystickButton(operator, PS4Controller.Button.kL2.value).whileTrue(new RevShooterWheels(shooterSubsystem, 0.75));

    new JoystickButton(operator, PS4Controller.Button.kTriangle.value).whileTrue(new AmpShoot(ampSubsystem, shooterSubsystem, 1));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value).whileTrue(new AmpShoot(ampSubsystem, shooterSubsystem, -1));

    new JoystickButton(operator, PS4Controller.Button.kOptions.value).whileTrue(new ClimbCommand(climberSubsystem, 0.5));
    new JoystickButton(operator, PS4Controller.Button.kShare.value).whileTrue(new ClimbCommand(climberSubsystem, -0.5));
    new JoystickButton(operator, PS4Controller.Button.kTouchpad.value).onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> LEDSubsystem.led0.setData(LEDCommand.setStripColor(27, 0, 0, 255))),
      new WaitCommand(1),
      new InstantCommand(() -> LEDSubsystem.led0.setData(LEDCommand.setStripColor(27, 0, 0, 0)))
    ));
  }

  public Command getAutonomousCommand() {
    System.out.println("Running autonomous...");
    String pathName = "Sample 1";
    // Follow Path
    return swerveSubsystem.getAutonomousCommand(pathName, true);
    // Follow Auto (Needs testing)
    // return swerveSubsystem.followAuto(pathName);

  }
}
