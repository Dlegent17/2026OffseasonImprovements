package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final PIDController aimController;

    public AutoAimTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        this.aimController = new PIDController(0.03, 0.0, 0.0);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (hasTarget) {
            double tx = LimelightHelpers.getTX("limelight");
            double rotationSpeed = aimController.calculate(tx, 0.0);
            turret.setTurretSpeed(-rotationSpeed); 
        } else {
            turret.setTurretSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurretSpeed(0.0);
    }
}