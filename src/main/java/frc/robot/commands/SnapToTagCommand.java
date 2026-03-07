package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;

public class SnapToTagCommand extends Command {
    private final SwerveSubsystem drivebase;
    private final VisionSwerveSystem vision;

    public SnapToTagCommand(SwerveSubsystem drivebase, VisionSwerveSystem vision) {
        this.drivebase = drivebase;
        this.vision = vision;
        
        // This tells the robot: "While this command is running, no other command 
        // (like the driver's joysticks) is allowed to use the drivebase!"
        addRequirements(drivebase); 
    }

    @Override
    public void execute() {
        // 1. Get the rotation speed from the Limelight PID in your Vision subsystem
        double rotSpeed = vision.getAimingRotationSpeed();

        // 2. Drive! 
        // Translation2d is set to (0,0) so we don't move forward/backward/left/right.
        // We only feed it the rotSpeed to spin in place.
        drivebase.drive(new Translation2d(0.0, 0.0), rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        // The command automatically finishes and releases the wheels 
        // when the robot is perfectly aligned.
        return vision.isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        // Tell the drivetrain to completely stop moving when the command finishes
        // or if the driver lets go of the button early.
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, false);
    }
}