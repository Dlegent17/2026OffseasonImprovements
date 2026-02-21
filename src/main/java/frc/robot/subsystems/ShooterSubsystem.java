package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax topFlywheel;
    private final SparkMax bottomFlywheel;
    private final SparkMax feederMotor;

    private boolean isShooterOn = false;
    private final double SHOOTER_SPEED = 0.8;

    @SuppressWarnings("removal")
    public ShooterSubsystem() {
        topFlywheel = new SparkMax(20, MotorType.kBrushless);
        bottomFlywheel = new SparkMax(21, MotorType.kBrushless);
        feederMotor = new SparkMax(22, MotorType.kBrushless);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.smartCurrentLimit(40);
        flywheelConfig.idleMode(IdleMode.kCoast);

        topFlywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig bottomConfig = new SparkMaxConfig();
        bottomConfig.apply(flywheelConfig);
        bottomConfig.inverted(true); 
        bottomFlywheel.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);
        feederConfig.idleMode(IdleMode.kBrake);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void toggleShooter() {
        isShooterOn = !isShooterOn;
        if (isShooterOn) {
            topFlywheel.set(SHOOTER_SPEED);
            bottomFlywheel.set(SHOOTER_SPEED);
        } else {
            topFlywheel.set(0.0);
            bottomFlywheel.set(0.0);
        }
    }

    public Command toggleShooterCommand() {
        return Commands.runOnce(this::toggleShooter, this);
    }

    public Command shootOneBallCommand() {
        return Commands.runEnd(
            () -> feederMotor.set(0.6),  
            () -> feederMotor.set(0.0),  
            this
        ).withTimeout(0.5); 
    }
}