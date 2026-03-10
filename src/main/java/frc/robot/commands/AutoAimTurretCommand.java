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
        this.aimController = new PIDController(0.015, 0.0, 0.0015);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // THE FIX: Reset the PID so it doesn't carry over old math
        aimController.reset();
    }

    @Override
public void execute() {
    if (LimelightHelpers.getTV("limelight")) {
        // Use TX because the Web UI is in Portrait mode
        double error = LimelightHelpers.getTX("limelight");
        
        // Calculate speed
        double speed = aimController.calculate(error, 0.0);
        
        // THE FIX: If it spins the wrong way, add the "-" here. 
        // If it was already negative, make it positive.
        turret.setTurretSpeed(speed); 
    } else {
        turret.setTurretSpeed(0.0);
    }
}

    @Override
    public void end(boolean interrupted) {
        turret.setTurretSpeed(0.0);
    }
}