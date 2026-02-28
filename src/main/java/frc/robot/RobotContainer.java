// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimTurretCommand;
import frc.robot.commands.SnapToTagCommand; // <-- FIX 1: ADDED THIS IMPORT
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.MechanismSubsystems.IntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystems.ShooterSubsystem;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;
import frc.robot.subsystems.MechanismSubsystems.IndexerSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
// Shooter and Turret Subsystems
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final TurretSubsystem turret = new TurretSubsystem();
  // The driver's controller
  final CommandXboxController driverXbox = new CommandXboxController(0);

// Just declare it, don't initialize it yet!
private final SendableChooser<Command> autoChooser;

  // The robot's subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //
  // Vision Swerve System 
  private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator());

  // Intake Subsystem
  public final IntakeSubsystem intake = new IntakeSubsystem();

  // --- NEW: Declared the Indexer Subsystem ---
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  

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
    // --- NEW: Setup PathPlanner FIRST so AutoBuilder doesn't crash ---
    drivebase.setupPathPlanner();

    // --- NEW: Moved NamedCommands above AutoBuilder so PathPlanner knows your commands ---
    // Intake Commands
    NamedCommands.registerCommand("DeployAndIntake", intake.intakeInCommand());
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intake.stopRollers()));
// Runs the indexer for exactly 0.5 seconds to push the ball into the flywheels
NamedCommands.registerCommand("FeedIndexer", indexer.feedToShooterCommand().withTimeout(0.5));
    // Shooter & Turret Commands
    NamedCommands.registerCommand("SpinUpShooter", Commands.runOnce(() -> shooter.toggleShooter()));
    NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> shooter.toggleShooter())); // Assuming it toggles off
    NamedCommands.registerCommand("AutoAim", new AutoAimTurretCommand(turret).withTimeout(1.5));

    // You can even register the whole sequence we built earlier as a single block!
NamedCommands.registerCommand("AimAndFireSequence", Commands.sequence(
    // 1. Aim the turret and rev the flywheels (using the Limelight distance)
    new AutoAimTurretCommand(turret).withTimeout(1.5),
    
    // 2. FIRE! (Push the ball into the flywheels)
    indexer.feedToShooterCommand().withTimeout(0.5),

    // 3. Turn the shooter off to save battery
    Commands.runOnce(() -> shooter.stopShooter())
));

    // --- INSIDE THE CONSTRUCTOR ---
    
    // This MUST be the only time autoChooser is assigned a value!
    autoChooser = AutoBuilder.buildAutoChooser();

    // Now you can use it, but you can't re-assign it.
    SmartDashboard.putData("Auto Choices", autoChooser);
    // Configure the trigger bindings
    configureBindings();
   DriverStation.silenceJoystickConnectionWarning(true);

    // Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    // Setup the Aim and Shoot routine
    // --- NEW: Updated the Auto routine to use the Indexer and explicit on/off states ---
    Command autoShootRoutine = Commands.sequence(
        Commands.runOnce(() -> shooter.setDynamicShooter(2.0)),
        new AutoAimTurretCommand(turret).withTimeout(1.5),
        indexer.feedToShooterCommand().withTimeout(0.5),
        Commands.runOnce(() -> shooter.stopShooter())
    );

    // Add it to your autonomous chooser
    autoChooser.addOption("Aim and Shoot", autoShootRoutine);
    
    // Send the chooser to the dashboard ONCE at the very end!
    // SmartDashboard.putData("Auto Choices", autoChooser); // --- NEW: Commented out to prevent duplicate dashboard errors ---
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
    
   // Smart Zero Gyro (start/menu Button)
        driverXbox.start().onTrue(Commands.runOnce(() -> {
    // Check if we are on the Red Alliance
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        // If on Red, facing away from the driver is 180 degrees
        drivebase.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(
            drivebase.getPose().getTranslation(), 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180)
        ));
    } else {
        // If on Blue, facing away from the driver is 0 degrees
        drivebase.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(
            drivebase.getPose().getTranslation(), 
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)
        ));
    }
}));

    
    // Snap To Tag (Y Button)
    driverXbox.b().whileTrue(new SnapToTagCommand(drivebase, visionSwerveSystem));

    // --- NEW: Indexer Unjam (A Button) ---
    driverXbox.a().whileTrue(indexer.reverseIndexerCommand());


    // --- SHOOTER BINDINGS ---
    
    // Right Trigger: Toggle the flywheels On and Off
    // The '0.5' means it activates when you pull the trigger halfway down
    // --- NEW: Updated to trigger the safe auto-shoot routine! ---
    Command triggerShootRoutine = Commands.sequence(
        Commands.runOnce(() -> shooter.setDynamicShooter(2.0)), 
        new AutoAimTurretCommand(turret).withTimeout(1.5),
        indexer.feedToShooterCommand().withTimeout(0.5),
        Commands.runOnce(() -> shooter.stopShooter())
    );
    driverXbox.rightTrigger(0.5).onTrue(triggerShootRoutine);

    // --- TURRET BINDINGS ---
    
    // Right Bumper: Hold to turn turret Right
    driverXbox.rightBumper().whileTrue(turret.turnRightCommand());

    // Left Bumper: Hold to turn turret Left
    driverXbox.leftBumper().whileTrue(turret.turnLeftCommand());

    // Squeeze Left Trigger: Drop down and spin.
    // Let go of Left Trigger: Automatically stow and stop.
    driverXbox.leftTrigger()
        .whileTrue(intake.deployAndIntakeCommand())
        .onFalse(intake.stowIntakeCommand());
    // --- OVERRIDE BUTTON ---
    // Force Pose Reset (X Button)
    driverXbox.x().onTrue(Commands.runOnce(() -> {
      // 1. Ask the vision system for a hard-reset pose
      Pose2d visionPose = visionSwerveSystem.getForceResetPose();
      
      // 2. If it actually saw a tag, force the drivetrain to jump to those coordinates
      if (visionPose != null) {
          drivebase.resetOdometry(visionPose);
      }
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public SwerveSubsystem getDrivebase()
  {
    return drivebase;
  }
}