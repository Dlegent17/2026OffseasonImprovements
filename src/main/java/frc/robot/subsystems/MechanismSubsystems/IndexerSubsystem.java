package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 

public class IndexerSubsystem extends SubsystemBase {
    
    private final SparkMax indexerMotor = new SparkMax(25, MotorType.kBrushless);

    @SuppressWarnings("removal")
    public IndexerSubsystem() {
        // Create the configuration object
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Use the IdleMode from the config package
        config.idleMode(IdleMode.kBrake);
        config.inverted(false); 

        // Apply to motor
        indexerMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void runForward() {
        indexerMotor.set(-1.0); 
    }

    public void runReverse() {        
        indexerMotor.set(1.0); 
    }

    public void stop() {
        indexerMotor.set(0.0);
    }

    // --- COMMANDS ---
    public Command feedToShooterCommand() {
        return this.runEnd(this::runForward, this::stop);
    }

    public Command reverseIndexerCommand() {
        return this.runEnd(this::runReverse, this::stop);
    }
}