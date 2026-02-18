// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SnapToTagCommand; // <-- FIX 1: ADDED THIS IMPORT
import frc.robot.subsystems.Limelight.VisionSwerveSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The driver's controller
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Auto Chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Vision Swerve System 
  // <-- FIX 2: Changed 'SwerveSubsystem' to 'drivebase'
  private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator());

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY(), // Forward/Back
                                                                () -> -driverXbox.getLeftX()) // Left/Right
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX()) // Turn
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
   * (Used for specific alignment commands if needed)
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings()
  {
    // --- DEFAULT DRIVE COMMAND ---
    // This sets the Left Stick to Drive, Right Stick to Turn (Angular Velocity)
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // --- DRIVER BUTTONS ---
    
    // Zero Gyro (Start Button)
    driverXbox.b().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Lock Wheels (X Button) - Useful for defense or staying still
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase));
    
    // Snap To Tag (Y Button)
    driverXbox.y().whileTrue(new SnapToTagCommand(drivebase, visionSwerveSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */