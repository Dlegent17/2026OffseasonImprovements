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
import frc.robot.commands.SnapToTagCommand; 
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

  // Vision Swerve System 
  private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator());

  // Intake Subsystem
  public final IntakeSubsystem intake = new IntakeSubsystem();

  // Indexer Subsystem
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY(), // Forward/Back
                                                                () -> -driverXbox.getLeftX()) // Left/Right
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX()) // Turn
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  public RobotContainer()
  {
    // --- Setup PathPlanner FIRST so AutoBuilder doesn't crash ---
    drivebase.setupPathPlanner();

    // Intake Commands
    NamedCommands.registerCommand("DeployAndIntake", intake.intakeInCommand());
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intake.stopRollers()));
    
    // Indexer Command
    NamedCommands.registerCommand("FeedIndexer", indexer.feedToShooterCommand().withTimeout(0.5));
    
    // Shooter & Turret Commands
    NamedCommands.registerCommand("SpinUpShooter", Commands.runOnce(() -> shooter.toggleShooter()));
    NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> shooter.toggleShooter())); 
    NamedCommands.registerCommand("AutoAim", new AutoAimTurretCommand(turret).withTimeout(1.5));

    NamedCommands.registerCommand("AimAndFireSequence", Commands.sequence(
        new AutoAimTurretCommand(turret).withTimeout(1.5),
        indexer.feedToShooterCommand().withTimeout(0.5),
        Commands.runOnce(() -> shooter.stopShooter())
    ));

    // --- NEW: Register the Multi-Shot Commands for Auto! ---
    NamedCommands.registerCommand("Shoot10Balls", shootMultipleBalls(10));
    NamedCommands.registerCommand("Shoot20Balls", shootMultipleBalls(20));

    // This MUST be the only time autoChooser is assigned a value!
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Choices", autoChooser);
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    Command autoShootRoutine = Commands.sequence(
        Commands.runOnce(() -> shooter.setDynamicShooter(2.0)),
        new AutoAimTurretCommand(turret).withTimeout(1.5),
        indexer.feedToShooterCommand().withTimeout(0.5),
        Commands.runOnce(() -> shooter.stopShooter())
    );

    autoChooser.addOption("Aim and Shoot", autoShootRoutine);
  }

  private void configureBindings()
  {
    // --- DEFAULT DRIVE COMMAND ---
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // --- DRIVER BUTTONS ---
    // Smart Zero Gyro (start/menu Button)
    driverXbox.start().onTrue(Commands.runOnce(() -> {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            drivebase.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(
                drivebase.getPose().getTranslation(), 
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180)
            ));
        } else {
            drivebase.resetOdometry(new edu.wpi.first.math.geometry.Pose2d(
                drivebase.getPose().getTranslation(), 
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)
            ));
        }
    }));
    
    // Snap To Tag (Y Button)
    driverXbox.b().whileTrue(new SnapToTagCommand(drivebase, visionSwerveSystem));

    // Indexer Unjam (A Button)
    driverXbox.a().whileTrue(indexer.reverseIndexerCommand());

    // --- SHOOTER BINDINGS ---
    Command triggerShootRoutine = Commands.sequence(
        Commands.runOnce(() -> shooter.setDynamicShooter(2.0)), 
        new AutoAimTurretCommand(turret).withTimeout(1.5),
        indexer.feedToShooterCommand().withTimeout(0.5),
        Commands.runOnce(() -> shooter.stopShooter())
    );
    driverXbox.rightTrigger(0.5).onTrue(triggerShootRoutine);

    // --- TURRET BINDINGS ---
    driverXbox.rightBumper().whileTrue(turret.turnRightCommand());
    driverXbox.leftBumper().whileTrue(turret.turnLeftCommand());

    // --- INTAKE BINDINGS ---
    driverXbox.leftTrigger()
        .whileTrue(intake.deployAndIntakeCommand())
        .onFalse(intake.stowIntakeCommand());

    // --- OVERRIDE BUTTON ---
    driverXbox.x().onTrue(Commands.runOnce(() -> {
      Pose2d visionPose = visionSwerveSystem.getForceResetPose();
      if (visionPose != null) {
          drivebase.resetOdometry(visionPose);
      }
    }));
  }

  // ==========================================
  // NEW METHOD: Generates a multi-shot sequence
  // ==========================================
  public Command shootMultipleBalls(int numberOfShots) {
      Command repeatedSequence = Commands.none(); 

      for (int i = 0; i < numberOfShots; i++) {
          repeatedSequence = repeatedSequence.andThen(
              new AutoAimTurretCommand(turret).withTimeout(1.0),
              indexer.feedToShooterCommand().withTimeout(0.5),
              Commands.waitSeconds(0.25) 
          );
      }
      return repeatedSequence.andThen(Commands.runOnce(() -> shooter.stopShooter()));
  }

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