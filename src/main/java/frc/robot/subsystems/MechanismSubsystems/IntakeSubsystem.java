package frc.robot.subsystems.MechanismSubsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final SparkMax frontRollerMotor;
    private final SparkMax backBeltMotor;
    private final SparkMax pivotMotor;

    // Absolute Encoder connected to roboRIO DIO
    private final DutyCycleEncoder pivotEncoder;

    // PLACE HOLDER ANGLE LIMITS - These are just guesses for now since we don't have the real robot or encoder values yet.
    // 0 degrees is theoretical fully stowed (up)
    // 90 degrees is theoretical fully deployed (down to the floor)
    private final double MAX_ANGLE_UP = 5.0;
    private final double MIN_ANGLE_DOWN = 85.0;

    @SuppressWarnings("removal")
    // Constructor initializes motors and encoder, and configures motor settings like current limits and idle modes.
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
        // This runs constantly. It puts your exact intake angle on the dashboard 
        // so you can easily read it and find your real limits later!
        SmartDashboard.putNumber("Intake Pivot Angle", getPivotAngle());
    }
    /**
     * Reads the absolute encoder and converts it to degrees.
     * DutyCycleEncoders return 0.0 to 1.0 by default, so we multiply by 360.
     */
    public double getPivotAngle() {
        // NOTE: Depending on how it's mounted, you might need to add an offset here later
        return pivotEncoder.get() * 360.0;
    }
// Set the speed of both the front roller and back belt motors at the same time for convenience.
    public void setRollerSpeed(double speed) {
        frontRollerMotor.set(speed);
        backBeltMotor.set(speed);
    }
// A simple helper method to stop all intake motors.
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

    /**
     * A Command that simultaneously drops the intake and runs the rollers to pull in a game piece.
     * Drops the intake to the floor and spins the rollers.
     * The soft limits in setPivotSpeed() will automatically stop the arm when it hits the floor!
     */
    public Command runIntakeCommand() {
        return this.runOnce(() -> 
            //frontRollerMotor.set(0.9)
            backBeltMotor.set(0.9)
            //setRollerSpeed(0.8); // Spin IN
            
        );
    }

    public Command getPivotDown() {
        return this.run(() -> {
            setPivotSpeed(-0.4);       
             }).until(() -> getPivotAngle() >= MIN_ANGLE_DOWN - 2.0); // Stop a little early to avoid hitting the floor hard!
    }

    /**
     * Pulls the intake back up into the robot and stops the rollers.
     * It finishes automatically when the absolute encoder says it has reached the top.
     */
    public Command stowIntakeCommand() {
        return this.run(() -> {
            setPivotSpeed(0.4);  // Drive UP (Positive)
            setRollerSpeed(0.0); // Stop rollers
        }).until(() -> getPivotAngle() <= MAX_ANGLE_UP + 2.0); // Stop the command when fully stowed
    }

    /**
     * Safely drives the pivot up or down, respecting the encoder limits.
     * @param speed positive for Up, negative for Down
     */
    public void setPivotSpeed(double speed) {
        double currentAngle = getPivotAngle();

        // CHECK 1: Are we trying to go too far Up?
        // (Assuming positive speed is Up, and smaller angle is Up)
        if (speed > 0 && currentAngle <= MAX_ANGLE_UP) {
            speed = 0.0; // Force stop!
        }
        
        // CHECK 2: Are we trying to go too far Down?
        // (Assuming negative speed is Down, and larger angle is Down)
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