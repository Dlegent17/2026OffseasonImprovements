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
import frc.robot.commands.SnapToTagCommand;
import frc.robot.subsystems.MechanismSubsystems.FloorSubsystem;
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
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final FloorSubsystem floor = new FloorSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    private final VisionSwerveSystem visionSwerveSystem = new VisionSwerveSystem(drivebase.getPoseEstimator(), turret);    
	@SuppressWarnings("unused")
    private final String limelightName = "limelight";
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
        NamedCommands.registerCommand("StopShooter", Commands.runOnce(() -> shooter.stopShooter())); 
        NamedCommands.registerCommand("FixedShooter", fixedTriggerShootRoutine);

        // 3. Build the Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Choices", autoChooser);

        // 4. Configure default autos and bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

    }
       
private final Command fixedTriggerShootRoutine = Commands.sequence(
    Commands.parallel(
        Commands.run(() -> shooter.setDynamicShooter(visionSwerveSystem.getDistanceToHubMeters()), shooter),
        new SnapToTagCommand(drivebase, visionSwerveSystem).withTimeout(1.5), indexer.feedToShooterCommand(), floor.fixedIntake()
    )
);
        
    private final Command passingTriggerShootRoutine = Commands.parallel(indexer.feedToShooterCommand(), floor.fixedIntake(), shooter.passingShooter());
         
    private final Command fixedStopShootRoutine = Commands.parallel(
        Commands.run(() -> indexer.stop(), indexer), 
        Commands.run(() -> floor.stopIntake(), floor),
        Commands.run(() -> shooter.stopShooterAndStartHoming(), shooter), 
        Commands.run(() -> intake.stopRollers(), intake));

    private final Command deployandIntakeCommand = 
    Commands.sequence(intake.getPivotDown(), intake.fixedRollers());

    private void configureBindings() {
            //SIMPLE FINAL BINDINGS PROBABLY
        driverXbox.leftTrigger().onTrue(deployandIntakeCommand);
        driverXbox.rightTrigger().onTrue(fixedTriggerShootRoutine);
        driverXbox.y().onTrue(fixedStopShootRoutine);
        
        // Default drive command with vision updates included
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity)
            .alongWith(Commands.run(() -> visionSwerveSystem.updateVision(
                drivebase.getHeading().getDegrees(), 
                drivebase.getTurnRate()
            ), visionSwerveSystem)));

     driverXbox.b().onTrue(Commands.runOnce(() -> {
    var alliance = DriverStation.getAlliance();
    
    double resetAngle = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) ? 0.0 : 180.0;
    
    // Get the robot's CURRENT position so we don't teleport it to the hub!
    Pose2d currentPose = drivebase.getPose(); // (Use whatever method your swerve uses to get current pose)
    
    // Reset the odometry with the CURRENT X/Y, but the NEW zeroed angle
    drivebase.resetOdometry(new Pose2d(
        currentPose.getX(), 
        currentPose.getY(), 
        Rotation2d.fromDegrees(resetAngle)
    ));
}));

         driverXbox.x().onTrue(Commands.runOnce(() -> {
           Pose2d visionPose = visionSwerveSystem.getForceResetPose();
            if (visionPose != null) {
                 drivebase.resetOdometry(visionPose);
              System.out.println("Odometry Reseeded Successfully!");
            } else {
                 System.out.println("Reseed Failed: No Tags Visible");
             }
         }));
         driverXbox.a().onTrue(new SnapToTagCommand(drivebase, visionSwerveSystem).withTimeout(1.5));
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
