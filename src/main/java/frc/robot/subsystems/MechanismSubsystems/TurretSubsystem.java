package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

/**
 * The TurretSubsystem controls the rotational movement of the shooter turret.
 * It uses a single SparkMax motor with soft limits to prevent the turret 
 * from spinning too far and ripping out its wires.
 */
public class TurretSubsystem extends SubsystemBase {

    // The motor controller that spins the turret
    private final SparkMax turretMotor;

    @SuppressWarnings("removal")
    public TurretSubsystem() {
        // Initialize the SparkMax on CAN ID 24 as a brushless motor (NEO/NEO 550)
        turretMotor = new SparkMax(24, MotorType.kBrushless);

        // Create a configuration object using the new REVLib 2025/2026 API
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        
        // Limit the motor to 30 Amps to prevent electrical brownouts or motor burnout
        turretConfig.smartCurrentLimit(30); 
        
        // Set to Brake mode so the turret stops immediately and holds its aim when power is removed
        turretConfig.idleMode(IdleMode.kBrake); 
        
        // --- SOFT LIMITS ---
        // These act as "virtual hard stops" based on the encoder. 
        // Once the motor hits 11.5 rotations, the SparkMax will automatically cut power 
        // in that direction to prevent the turret from snapping its cables.
        turretConfig.softLimit.forwardSoftLimitEnabled(true);
        turretConfig.softLimit.forwardSoftLimit(11.5); 

        turretConfig.softLimit.reverseSoftLimitEnabled(true);
        turretConfig.softLimit.reverseSoftLimit(-11.5); 

        // Apply this entire configuration to the physical motor controller.
        // It resets old parameters safely and saves the new ones to the motor's flash memory.
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the raw speed of the turret motor.
     * @param speed The motor power from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }

    /**
     * A Command that manually rotates the turret to the right at 75% power.
     * When the command ends (e.g., the driver lets go of the button), the motor stops.
     */
    public Command turnRightCommand() {
        return this.runEnd(() -> setTurretSpeed(0.75), () -> setTurretSpeed(0.0));
    }

    /**
     * A Command that manually rotates the turret to the left at 75% power.
     * When the command ends (e.g., the driver lets go of the button), the motor stops.
     */
    public Command turnLeftCommand() {
        return this.runEnd(() -> setTurretSpeed(-0.75), () -> setTurretSpeed(0.0));
    }
}
