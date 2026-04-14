package frc.robot.subsystems.MechanismSubsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax frontRollerMotor;
    private final SparkMax pivotMotor;

    // Absolute Encoder connected to roboRIO DIO
    private final RelativeEncoder pivotEncoder;

    // PLACE HOLDER ANGLE LIMITS - These are just guesses for now since we don't have the real robot or encoder values yet.
    // 0.0 degrees is theoretical fully stowed (up)
    // 90.0 degrees is theoretical fully deployed (down to the floor)
    private final double StowedPosition = 0.0;
    private final double DeployedPostion = 71.5;

    @SuppressWarnings("removal")
    // Constructor initializes motors and encoder, and configures motor settings like current limits and idle modes.
    public IntakeSubsystem() {
        frontRollerMotor = new SparkMax(15, MotorType.kBrushless);
        pivotMotor = new SparkMax(17, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        
        // Configuring Front Roller Motor
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.idleMode(IdleMode.kCoast);
        frontRollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Pivot Motor with Brake Mode and Current Limit
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(30);
        pivotConfig.idleMode(IdleMode.kBrake); 
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This runs constantly. It find the turt deployed postion
        SmartDashboard.putNumber("Intake Pivot Rotations", getPivotPosition());
    }
/**
     * Reads the relative encoder. 
     * By default, this returns the number of motor rotations.
     */
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
public void setFrontRollerSpeed(double speed) {
        frontRollerMotor.set(speed);
    }

    public void stopRollers() {
        frontRollerMotor.set(0.0);
    }

    public void runRollers() {
        frontRollerMotor.set(-1.0);
    }

    public Command fixedRollers() {
        return this.runEnd(this::runRollers, this::stopRollers);
    }

    public Command tempIntakeAndFloorCommand() {
        return this.runEnd(() -> setFrontRollerSpeed(-0.7), this::stopRollers);
    }

    // --- Pivot Commands ---

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public Command intakeInCommand() {
        return this.runEnd(() -> setPivotSpeed(0.3), () -> setPivotSpeed(0.0));
    }

    public Command intakeOutCommand() {
        return this.runEnd(() -> setPivotSpeed(-0.3), () -> setPivotSpeed(0.0));
    }

    /**
     * Drives the intake down to the floor.
     * Stops automatically when it hits the DEPLOYED_POSITION.
     */
    public Command getPivotDown() {
        return this.run(() -> {
            pivotMotor.set(0.3); 
        })
        .until(() -> getPivotPosition() >= DeployedPostion)
        .andThen(() -> pivotMotor.set(0.0)).andThen(fixedRollers()); 
    }

    /**
     * Pulls the intake back up into the robot.
     * Stops automatically when it reaches the STOWED_POSITION (0.0).
     */
    public Command stowIntakeCommand() {
        return this.run(() -> {
            frontRollerMotor.set(0.0);
            pivotMotor.set(-0.5); 
        })
        .until(() -> getPivotPosition() <= StowedPosition)
        .andThen(() -> pivotMotor.set(0.0));
    }

    public Command manualPivotCommand(java.util.function.DoubleSupplier joystickAxis) {
        return this.run(() -> {
            double stickValue = joystickAxis.getAsDouble();
            if (Math.abs(stickValue) < 0.1) {
                pivotMotor.set(0.0);
            } else {
                pivotMotor.set(stickValue * 0.4); 
            }
        });
    }
}
