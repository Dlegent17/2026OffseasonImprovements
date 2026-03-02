package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax frontRollerMotor;
    private final SparkMax backBeltMotor;
    private final SparkMax pivotMotor;

    // Absolute Encoder connected to roboRIO DIO
    private final DutyCycleEncoder pivotEncoder;

    // --- PLACEHOLDER LIMITS ---
    // We will tune these later. For now, let's pretend:
    // 0 degrees is fully stowed (up)
    // 90 degrees is fully deployed (down to the floor)
    private final double MAX_ANGLE_UP = 5.0;   // Don't crash into the chassis
    private final double MIN_ANGLE_DOWN = 85.0; // Don't smash into the floor

    @SuppressWarnings("removal")
    public IntakeSubsystem() {
        frontRollerMotor = new SparkMax(15, MotorType.kBrushless);
        backBeltMotor = new SparkMax(16, MotorType.kBrushless);
        pivotMotor = new SparkMax(17, MotorType.kBrushless);

        // Initialize Encoder on roboRIO DIO Port 0
        pivotEncoder = new DutyCycleEncoder(0);

        // 
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.idleMode(IdleMode.kCoast);

        frontRollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig backBeltConfig = new SparkMaxConfig();
        backBeltConfig.apply(rollerConfig);
        backBeltConfig.inverted(true);
        backBeltMotor.configure(backBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Pivot Motor with Brake Mode and Current Limit
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(40);
        pivotConfig.idleMode(IdleMode.kBrake); 
        
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This runs constantly. It puts exact intake angle on the dashboard 
        // so you can easily read it and find your real limits later!
        SmartDashboard.putNumber("Intake Pivot Angle", getPivotAngle());
    }

    // Encoder Logic
     * Reads the absolute encoder and converts it to degrees.
     * DutyCycleEncoders return 0.0 to 1.0 by default, so we multiply by 360.
     */
    public double getPivotAngle() {
        // NOTE: Depending on how it's mounted, you might need to add an offset here later
        return pivotEncoder.get() * 360.0;
    }

    //
    // Roller Controller
    public void setRollerSpeed(double speed) {
        frontRollerMotor.set(speed);
        backBeltMotor.set(speed);
    }

    public void stopRollers() {
        frontRollerMotor.set(0.0);
        backBeltMotor.set(0.0);
    }

    public Command intakeInCommand() {
        return this.runEnd(() -> setRollerSpeed(0.8), this::stopRollers);
    }

    public Command intakeOutCommand() {
        return this.runEnd(() -> setRollerSpeed(-0.6), this::stopRollers);
    }
    // Command so the intake goes to the flow and spins when a button is pressed
    public Command deployAndIntakeCommand() {
        return this.run(() -> {
            setPivotSpeed(-0.4); // Drive Down (Negative)
            setRollerSpeed(0.8); // Spin In
        });
    }

    /**
     * Pulls the intake back up into the robot and stops the rollers.
     * It finishes automatically when the absolute encoder says it has reached the top.
     */
    public Command stowIntakeCommand() {
        return this.run(() -> {
            setPivotSpeed(0.4);  // Drive UP (Positive)
            setRollerSpeed(0.0); // Stop rollers
        }).until(() -> getPivotAngle() <= MAX_ANGLE_UP); // Stop the command when fully stowed!
    }

// Pivot Controls with limits to how far the intake can go up and down

    /**
     * Safely drives the pivot up or down, respecting the encoder limits.
     * @param speed positive for UP, negative for DOWN
     */
    public void setPivotSpeed(double speed) {
        double currentAngle = getPivotAngle();

        // CHECK 1: Are we trying to go too far UP?
        // (Assuming positive speed is UP, and smaller angle is UP)
        if (speed > 0 && currentAngle <= MAX_ANGLE_UP) {
            speed = 0.0; // Force stop!
        }
        
        // CHECK 2: Are we trying to go too far DOWN?
        // (Assuming negative speed is DOWN, and larger angle is DOWN)
        if (speed < 0 && currentAngle >= MIN_ANGLE_DOWN) {
            speed = 0.0; // Force stop!
        }

        pivotMotor.set(speed);
    }

    public Command manualPivotCommand(java.util.function.DoubleSupplier joystickAxis) {
        return this.run(() -> {
            double stickValue = joystickAxis.getAsDouble();
            if (Math.abs(stickValue) < 0.1) {
                setPivotSpeed(0.0);
            } else {
                setPivotSpeed(stickValue * 0.4); 
            }
        });
    }
}
