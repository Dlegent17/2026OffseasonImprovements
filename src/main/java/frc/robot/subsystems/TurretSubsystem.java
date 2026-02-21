package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor;

    @SuppressWarnings("removal")
    public TurretSubsystem() {
        turretMotor = new SparkMax(23, MotorType.kBrushless);

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.smartCurrentLimit(30); 
        turretConfig.idleMode(IdleMode.kBrake); 
        
        turretConfig.softLimit.forwardSoftLimitEnabled(true);
        turretConfig.softLimit.forwardSoftLimit(40.0); 

        turretConfig.softLimit.reverseSoftLimitEnabled(true);
        turretConfig.softLimit.reverseSoftLimit(-40.0); 

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }

    public Command turnRightCommand() {
        return this.runEnd(() -> setTurretSpeed(0.25), () -> setTurretSpeed(0.0));
    }

    public Command turnLeftCommand() {
        return this.runEnd(() -> setTurretSpeed(-0.25), () -> setTurretSpeed(0.0));
    }
}