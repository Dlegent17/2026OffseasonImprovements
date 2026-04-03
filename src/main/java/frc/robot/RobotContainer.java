package frc.robot;
 
import java.io.File;
 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SnapToTagCommand;
import frc.robot.subsystems.MechanismSubsystems.FloorSubsystem;
import frc.robot.subsystems.MechanismSubsystems.IndexerSubsystem;
import frc.robot.subsystems.MechanismSubsystems.IntakeSubsystem;
import frc.robot.subsystems.MechanismSubsystems.ShooterSubsystem;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;
 
import swervelib.SwerveInputStream;
 
public class RobotContainer {
 
    // =========================================================
    // SUBSYSTEMS
    // =========================================================
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ShooterSubsystem shooter   = new ShooterSubsystem();
    public final TurretSubsystem  turret    = new TurretSubsystem();
    public final IntakeSubsystem  intake    = new IntakeSubsystem();
    public final FloorSubsystem   floor     = new FloorSubsystem();
    public final IndexerSubsystem indexer   = new IndexerSubsystem();
 
    private final VisionSwerveSystem visionSwerveSystem =
        new VisionSwerveSystem(drivebase.getPoseEstimator(), turret);
 
    @SuppressWarnings("unused")
    private final String limelightName = "limelight";
 
    // =========================================================
    // CONTROLLERS
    // =========================================================
    final CommandXboxController driverXbox = new CommandXboxController(0);
 
    // =========================================================
    // AUTO
    // =========================================================
    private final SendableChooser<Command> autoChooser;
 
    // =========================================================
    // DRIVE INPUT STREAM
    // =========================================================
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
 
    // =========================================================
    // COMPOSED COMMANDS
    // =========================================================
 
    /** Right Trigger — Snap to hub, spin up shooter, feed balls. */
    private final Command shootCommand = Commands.parallel(
        Commands.run(() -> shooter.setDynamicShooter(visionSwerveSystem.getDistanceToHubMeters()), shooter),
        new SnapToTagCommand(drivebase, visionSwerveSystem).withTimeout(1.5),
        indexer.feedToShooterCommand(),
        floor.fixedIntake()
    );
 
    /** Left Trigger — Deploy intake pivot then spin rollers. */
    private final Command deployAndIntakeCommand =
        Commands.sequence(intake.getPivotDown(), intake.fixedRollers());
 
    /** B Button — Stow the intake back up. */
    private final Command stowIntakeCommand = intake.stowIntakeCommand();
 
    /** A Button — Passing mode: fixed mid-power shot. */
    private final Command passingShootCommand = Commands.parallel(
        indexer.feedToShooterCommand(),
        floor.fixedIntake(),
        shooter.passingShooter()
    );
 
    /** Y Button — Stop all mechanisms and home the hood. */
    private final Command stopEverythingCommand = Commands.parallel(
        Commands.run(() -> indexer.stop(),                  indexer),
        Commands.run(() -> floor.stopIntake(),              floor),
        Commands.run(() -> shooter.stopShooterAndStartHoming(), shooter),
        Commands.run(() -> intake.stopRollers(),            intake)
    );
 
    /** X Button — Reset heading relative to alliance wall. */
    private final Command zeroHeadingCommand = Commands.runOnce(() -> {
        var alliance = DriverStation.getAlliance();
        double resetAngle = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? 0.0 : 180.0;
        Pose2d current = drivebase.getPose();
        drivebase.resetOdometry(new Pose2d(current.getX(), current.getY(), Rotation2d.fromDegrees(resetAngle)));
    });
 
    /** D-Pad Up — Run floor belt forward only (no intake pivot or indexer). */
    private final Command floorForwardCommand = floor.fixedIntake();
 
    /** D-Pad Down — Run floor belt in reverse only. */
    private final Command floorReverseCommand = floor.reverseIntake();
 
    // Named command for PathPlanner
    private final Command fixedShooterAutoCommand = Commands.parallel(
        Commands.run(() -> shooter.setDynamicShooter(visionSwerveSystem.getDistanceToHubMeters()), shooter),
        new SnapToTagCommand(drivebase, visionSwerveSystem).withTimeout(1.5),
        indexer.feedToShooterCommand(),
        floor.fixedIntake()
    );
 
    // =========================================================
    // CONSTRUCTOR
    // =========================================================
    public RobotContainer() {
        drivebase.setupPathPlanner();
 
        NamedCommands.registerCommand("StopShooter",   Commands.runOnce(() -> shooter.stopShooter()));
        NamedCommands.registerCommand("FixedShooter",  fixedShooterAutoCommand);
 
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Choices", autoChooser);
 
        configureDefaultCommands();
        configureBindings();
 
        DriverStation.silenceJoystickConnectionWarning(true);
    }
 
    // =========================================================
    // DEFAULT COMMANDS
    // =========================================================
    private void configureDefaultCommands() {
        drivebase.setDefaultCommand(
            drivebase.driveFieldOriented(driveAngularVelocity)
                .alongWith(Commands.run(() -> visionSwerveSystem.updateVision(
                    drivebase.getHeading().getDegrees(),
                    drivebase.getTurnRate()
                ), visionSwerveSystem))
        );
    }
 
    // =========================================================
    // BUTTON BINDINGS
    // Change bindings here — one place, clearly labeled.
    // =========================================================
    private void configureBindings() {
 
        // --- Right Trigger: SHOOT ---
        driverXbox.rightTrigger().whileTrue(shootCommand);
 
        // --- Left Trigger: DEPLOY & INTAKE ---
        driverXbox.leftTrigger().onTrue(deployAndIntakeCommand);
 
        // --- B Button: STOW INTAKE ---
        driverXbox.b().onTrue(stowIntakeCommand);
 
        // --- A Button: PASSING SHOT ---
        driverXbox.a().whileTrue(passingShootCommand);
 
        // --- Y Button: STOP EVERYTHING ---
        driverXbox.y().onTrue(stopEverythingCommand);
 
        // --- X Button: ZERO HEADING ---
        driverXbox.x().onTrue(zeroHeadingCommand);
 
        // --- D-Pad Up: RUN FLOOR FORWARD ---
        driverXbox.povUp().whileTrue(floorForwardCommand);
 
        // --- D-Pad Down: RUN FLOOR REVERSE ---
        driverXbox.povDown().whileTrue(floorReverseCommand);
    }
 
    // =========================================================
    // PUBLIC ACCESSORS
    // =========================================================
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
 
