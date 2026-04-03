package frc.robot.subsystems.MechanismSubsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorSubsystem extends SubsystemBase {

    private final SparkMax backBeltMotor;

    @SuppressWarnings("removal")
    public FloorSubsystem() {
        backBeltMotor = new SparkMax(16, MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.idleMode(IdleMode.kCoast);

        SparkMaxConfig backBeltConfig = new SparkMaxConfig();
        backBeltConfig.apply(rollerConfig);
        backBeltConfig.inverted(true);
        backBeltMotor.configure(backBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {}

    // --- Motor Control ---

    public void runIntake() {
        backBeltMotor.set(-0.2);
    }

    public void runIntakeReverse() {
        backBeltMotor.set(0.2);
    }

    public void stopIntake() {
        backBeltMotor.set(0.0);
    }

    public void stopRollers() {
        backBeltMotor.set(0.0);
    }

    // --- Commands ---

    /** Runs the floor belt forward (toward indexer). */
    public Command fixedIntake() {
        return this.runEnd(this::runIntake, this::stopIntake);
    }

    /** Runs the floor belt in reverse (eject direction). */
    public Command reverseIntake() {
        return this.runEnd(this::runIntakeReverse, this::stopIntake);
    }
}
