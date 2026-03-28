package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystems.VisionSwerveSystem;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;

public class SnapToTagCommand extends Command {
    private final SwerveSubsystem drivebase;
    private final VisionSwerveSystem vision;

    public SnapToTagCommand(SwerveSubsystem drivebase, VisionSwerveSystem vision) {
        this.drivebase = drivebase;
        this.vision = vision;
        
        // This ensures the driver's manual joystick input is disabled 
        // while the robot is auto-aligning.
        addRequirements(drivebase); 
    }

    @Override
    public void initialize() {
        // Optional: You could reset PID constants here if needed
    }

    @Override
    public void execute() {
        // 1. Get the calculated rotation speed from the Vision Subsystem
        double rotSpeed = vision.getAimingRotationSpeed();

        // 2. Deadband / Minimum Power Check
        // If the PID output is tiny (e.g., 0.01), the motors won't move.
        // We only drive if the rotation speed is significant enough to overcome friction.
        if (Math.abs(rotSpeed) < 0.05) {
            rotSpeed = 0.0;
        }

        // 3. Command the drivebase
        // Translation is (0,0) because we only want to rotate.
        // 'true' indicates field-relative driving.
        drivebase.drive(new Translation2d(0.0, 0.0), rotSpeed, true);
    }

  @Override
public boolean isFinished() {
    // Finish if we are perfectly aligned OR if we completely lose the tag 
    // (You can remove the getTV check if you want it to blindly trust odometry once it starts)
    return vision.isAligned() || !LimelightHelpers.getTV("limelight");
}

    @Override
    public void end(boolean interrupted) {
        // Stop the robot immediately when the command ends or is cancelled
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, false);
    }
}
