// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

// Commands & Constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAimTurretCommand;
import frc.robot.commands.SnapToTagCommand;
import frc.robot.subsystems.MechanismSubsystems.IndexerSubsystem;
import frc.robot.subsystems.MechanismSubsystems.IntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystems.ShooterSubsystem;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;

// Subsystems
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;

// Third-Party Libraries
import swervelib.SwerveInputStream;

public class RobotContainer {

    // --- Subsystems ---
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();
    // public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator(), turret);    

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

   

    // --- Constructor ---
    public RobotContainer() {
        
        // 1. Setup PathPlanner
        drivebase.setupPathPlanner();

        // 2. Register Named Commands
        NamedCommands.registerCommand("FeedIndexer", indexer.feedToShooterCommand().withTimeout(0.5));
        NamedCommands.registerCommand("SpinUpShooter", Commands.runOnce(() -> shooter.toggleShooter()));
        NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> shooter.stopShooter())); 
        NamedCommands.registerCommand("AutoAim", new AutoAimTurretCommand(turret).withTimeout(1.5));
        NamedCommands.registerCommand("AimAndFireSequence", triggerShootRoutine);

        NamedCommands.registerCommand("Shoot10Balls", shootMultipleBalls(10));
        NamedCommands.registerCommand("Shoot20Balls", shootMultipleBalls(20));
        NamedCommands.registerCommand("Shoot30Balls", shootMultipleBalls(30));

        // 3. Build the Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Choices", autoChooser);

        // 4. Configure default autos and bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

        // --- THE ADDITION: Auto-Seed Odometry while Disabled ---
        // This runs once per second while sitting behind the glass.
        // If the camera sees a tag, it automatically fixes the robot's field position.
        Commands.repeatingSequence(
            Commands.runOnce(() -> {
                if (DriverStation.isDisabled()) {
                    Pose2d visionPose = visionSwerveSystem.getForceResetPose();
                    if (visionPose != null) {
                        drivebase.resetOdometry(visionPose);
                    }
                }
            }),
            Commands.waitSeconds(1.0)
        ).ignoringDisable(true).schedule();
    }
private final Command triggerShootRoutine = Commands.sequence(
    // Step 1: Aim and Spin up for 1.5 seconds
    Commands.parallel(
        Commands.run(() -> shooter.setDynamicShooter(visionSwerveSystem.getDistanceToHubMeters()), shooter),
        new AutoAimTurretCommand(turret)
    ).withTimeout(1.5),

    // Step 2: Fire while maintaining speed for 0.5 seconds
    Commands.parallel(
        Commands.run(() -> shooter.setDynamicShooter(visionSwerveSystem.getDistanceToHubMeters()), shooter),
        new AutoAimTurretCommand(turret),
        indexer.feedToShooterCommand()
    ).withTimeout(0.5),

    // Step 3: Hard Stop
    Commands.runOnce(() -> shooter.stopShooter(), shooter)
);

    private void configureBindings() {
        
        
        // Default drive command with vision updates included
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity)
            .alongWith(Commands.run(() -> visionSwerveSystem.updateVision(
                drivebase.getHeading().getDegrees(), 
                drivebase.getTurnRate()
            ), visionSwerveSystem)));

        // A Button: Smart Zero Gyro & Pose based on Alliance Color
        driverXbox.a().onTrue(Commands.runOnce(() -> {
            var alliance = DriverStation.getAlliance();
            double xPos = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? 12.51 : 4.03;
            double resetAngle = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? 180.0 : 0.0;
            
            drivebase.resetOdometry(new Pose2d(
                xPos, 4.035, 
                Rotation2d.fromDegrees(resetAngle)
            ));
        }));
        
        // B Button: Snap To Tag (Chassis Aim)
        driverXbox.b().whileTrue(new SnapToTagCommand(drivebase, visionSwerveSystem));

        // In configureBindings()
        driverXbox.rightTrigger(0.5).onTrue(triggerShootRoutine);

        // Right/Left Bumpers: Manual Turret Turn
        driverXbox.rightBumper().whileTrue(turret.turnRightCommand());
        driverXbox.leftBumper().whileTrue(turret.turnLeftCommand());

        // --- THE ADDITION: X Button - Manual Force Pose Reset ---
        // This allows the driver to manually fix a "lost" robot icon by looking at a tag.
        driverXbox.x().onTrue(Commands.runOnce(() -> {
            Pose2d visionPose = visionSwerveSystem.getForceResetPose();
            if (visionPose != null) {
                drivebase.resetOdometry(visionPose);
                System.out.println("Odometry Reseeded Successfully!");
            } else {
                System.out.println("Reseed Failed: No Tags Visible");
            }
        })); 

        // Ferry Pass Sequence (kept for your reference)
        Command ferrySequence = Commands.sequence(
            Commands.parallel(
                Commands.run(() -> shooter.setFerryMode(), shooter), 
                new AutoAimTurretCommand(turret)
            ).withTimeout(1.0),
            
            Commands.parallel(
                Commands.run(() -> shooter.setFerryMode(), shooter),
                indexer.feedToShooterCommand()
            ).withTimeout(0.5),

            Commands.runOnce(() -> shooter.stopShooter(), shooter)
        );
        
        // driverXbox.y().onTrue(ferrySequence);
    }

    // --- Helper Methods ---
    
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