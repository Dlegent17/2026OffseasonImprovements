package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;

public class AutoAimTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final PIDController aimPID = new PIDController(0.05, 0.0, 0.002); // Added a tiny 'D' to stop overshoot

    public AutoAimTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
        
        // Tolerance: Stop moving if we are within 0.5 degrees of center
        aimPID.setTolerance(0.5);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight")) {
            // 1. Get the horizontal error
            double error = LimelightHelpers.getTY("limelight");

            // 2. Calculate PID speed
            double speed = aimPID.calculate(error, 0.0);

            // 3. THE FIX: The Negative Sign
            // If the turret still runs to the edge, REMOVE the minus sign.
            // If it already didn't have one, ADD it.
            turret.setTurretSpeed(speed); 
            
        } else {
            // If we lose the target, stop moving so we don't spin wildly
            turret.setTurretSpeed(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // This command only finishes if it's perfectly on target
        // But in your Shoot Sequence, the .withTimeout(1.5) will override this!
        return aimPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurretSpeed(0.0);
    }
}