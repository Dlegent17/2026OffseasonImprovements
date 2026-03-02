// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

// WPILib Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Subsystems
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;
import frc.robot.subsystems.MechanismSubsystems.IntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystems.ShooterSubsystem;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;
import frc.robot.subsystems.MechanismSubsystems.IndexerSubsystem;

// Commands & Constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimTurretCommand;
import frc.robot.commands.SnapToTagCommand; 

// Third-Party Libraries
import swervelib.SwerveInputStream;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

    // --- Subsystems ---
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator());
    
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();

    // --- Controllers & Choosers ---
    final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;

    // --- Drive Input Streams ---
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(), // Forward/Back
            () -> -driverXbox.getLeftX()) // Left/Right
        .withControllerRotationAxis(() -> -driverXbox.getRightX()) // Turn
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
        .headingWhile(true);

    // --- Constructor ---
    public RobotContainer() {
        
        // 1. Setup PathPlanner FIRST so AutoBuilder doesn't crash
        drivebase.setupPathPlanner();

        // 2. Register Named Commands for PathPlanner
        NamedCommands.registerCommand("DeployAndIntake", intake.intakeInCommand());
        NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intake.stopRollers()));
        
        NamedCommands.registerCommand("FeedIndexer", indexer.feedToShooterCommand().withTimeout(0.5));
        
        NamedCommands.registerCommand("SpinUpShooter", Commands.runOnce(() -> shooter.toggleShooter()));
        NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> shooter.stopShooter())); 
        NamedCommands.registerCommand("AutoAim", new AutoAimTurretCommand(turret).withTimeout(1.5));

        NamedCommands.registerCommand("AimAndFireSequence", Commands.sequence(
            new AutoAimTurretCommand(turret).withTimeout(1.5),
            indexer.feedToShooterCommand().withTimeout(0.5),
            Commands.runOnce(() -> shooter.stopShooter())
        ));

        NamedCommands.registerCommand("Shoot10Balls", shootMultipleBalls(10));
        NamedCommands.registerCommand("Shoot20Balls", shootMultipleBalls(20));

        // 3. Build the Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Choices", autoChooser);

        // 4. Configure default autos and bindings
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

    // --- Controller Bindings ---
    private void configureBindings() {
        
        // Default drive command
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

        // Start/Menu Button: Smart Zero Gyro based on Alliance Color
        driverXbox.start().onTrue(Commands.runOnce(() -> {
            var alliance = DriverStation.getAlliance();
            // If Red Alliance, facing away is 180 degrees. If Blue, facing away is 0 degrees.
            double resetAngle = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? 180.0 : 0.0;
            
            drivebase.resetOdometry(new Pose2d(
                drivebase.getPose().getTranslation(), 
                Rotation2d.fromDegrees(resetAngle)
            ));
        }));
        
        // Y Button: Snap To Tag 
        driverXbox.b().whileTrue(new SnapToTagCommand(drivebase, visionSwerveSystem));

        // A Button: Indexer Unjam
        driverXbox.a().whileTrue(indexer.reverseIndexerCommand());

        // Right Trigger (Half Pull): Auto Shoot Routine
        Command triggerShootRoutine = Commands.sequence(
            Commands.runOnce(() -> shooter.setDynamicShooter(2.0)), 
            new AutoAimTurretCommand(turret).withTimeout(1.5),
            indexer.feedToShooterCommand().withTimeout(0.5),
            Commands.runOnce(() -> shooter.stopShooter())
        );
        driverXbox.rightTrigger(0.5).onTrue(triggerShootRoutine);

        // Right/Left Bumpers: Manual Turret Turn
        driverXbox.rightBumper().whileTrue(turret.turnRightCommand());
        driverXbox.leftBumper().whileTrue(turret.turnLeftCommand());

        // Left Trigger: Deploy and Intake (Stows on release)
        driverXbox.leftTrigger()
            .whileTrue(intake.deployAndIntakeCommand())
            .onFalse(intake.stowIntakeCommand());

        // X Button: Force Pose Reset to Vision coordinates
        driverXbox.x().onTrue(Commands.runOnce(() -> {
            Pose2d visionPose = visionSwerveSystem.getForceResetPose();
            if (visionPose != null) {
                drivebase.resetOdometry(visionPose);
            }
        }));
    }

    // --- Helper Methods ---
    
    /** Generates a multi-shot sequence for PathPlanner Autos */
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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public SwerveSubsystem getDrivebase() {
        return drivebase;
    }
}
