package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;

public class AutoAimTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final PIDController aimController;

    public AutoAimTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        // P-value set to 0.02 to prevent the violent overshoot we saw earlier!
        this.aimController = new PIDController(0.02, 0.0, 0.0);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Checks if the Limelight currently sees an AprilTag
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (hasTarget) {
            // THE FIX: Back to 'tx' because the Limelight Web Dashboard already rotated the camera feed!
            double tx = LimelightHelpers.getTX("limelight"); 
            
            // Calculate the speed to reach 0.0 (centered on the crosshair)
            double rotationSpeed = aimController.calculate(tx, 0.0);
            
            // Apply the speed to the motor. 
            // NOTE: If it spins away from the target and hits the limit, just change this to: -rotationSpeed
            turret.setTurretSpeed(rotationSpeed); 
        } else {
            // If it loses the target, stop spinning
            turret.setTurretSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Safely stop the motor when the command finishes or gets canceled
        turret.setTurretSpeed(0.0);
    }
}