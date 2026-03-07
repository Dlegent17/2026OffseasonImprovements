package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
    
    // Defining Motors
    private final SparkMax rightFlywheel = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax leftFlywheel = new SparkMax(19, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(20, MotorType.kBrushless); 

    // Defining Hood sensor and control
    private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0);

    // Making Safety Limits (tune later)
    private final double HOOD_MIN_ANGLE = 0.10; // Bottom hard stop
    private final double HOOD_MAX_ANGLE = 0.45; // Top hard stop

    // The Interpolating Map
    private final InterpolatingDoubleTreeMap powerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    
    @SuppressWarnings("removal")
    public ShooterSubsystem() {

        // REV Lib motor configurations
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // 1. Flywheel Power (Percentage 0.0 to 1.0)
        powerMap.put(1.5, 0.45); 
        powerMap.put(3.0, 0.75); 
        powerMap.put(5.0, 1.00); 

        // 2. Hood Angle (Absolute Encoder Position)
        hoodMap.put(1.5, 0.12);  // Close shot: Hood mostly down
        hoodMap.put(3.0, 0.25);  // Mid shot: Hood half up
        hoodMap.put(5.0, 0.40);  // Far shot: Hood fully raised
    }

    // ==========================================
    // SHOOTER MODES
    // ==========================================
    
    /** Looks at the distance, checks the maps, and automatically adjusts the flywheels and hood */
    public void setDynamicShooter(double distanceToHubMeters) {
        double targetPower = powerMap.get(distanceToHubMeters);
        double targetHoodPosition = hoodMap.get(distanceToHubMeters);

        // Apply Power to Flywheels
        rightFlywheel.set(targetPower);
        leftFlywheel.set(targetPower);

        // Safely move the hood
        setHoodAngle(targetHoodPosition);
    }

    /** * FERRY MODE: Bypasses the distance map and blasts the game piece across the field.
     * Sets flywheels to 100% power and the hood to a high arcing angle.
     */
    public void setFerryMode() {
        rightFlywheel.set(1.0);
        leftFlywheel.set(1.0);
        
        // Sets the hood to the absolute maximum safe arc
        setHoodAngle(HOOD_MAX_ANGLE); 
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        leftFlywheel.set(0);
        
        // Drop the hood back to the bottom when the flywheels turn off
        setHoodAngle(HOOD_MIN_ANGLE);
    }

    /** Simple toggle for testing flywheels manually */
    public void toggleShooter() {
        if (rightFlywheel.get() > 0.1) { stopShooter(); } 
        else { rightFlywheel.set(0.5); leftFlywheel.set(-0.5); }
    }

    // ==========================================
    // HOOD CONTROL HELPER
    // ==========================================

    /** Calculates the PID to safely move the hood to a specific angle */
    public void setHoodAngle(double targetAngle) {
        // Clamp the requested hood position so it NEVER exceeds your mechanical limits
        double safeTarget = MathUtil.clamp(targetAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        // Calculate PID for the Hood and apply motor power
        double currentHoodPosition = hoodAbsoluteEncoder.get();
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
        
        // Limit the max speed of the hood so it doesn't snap violently
        hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.4, 0.4); 
        hoodMotor.set(hoodMotorPower);
    }

    @Override
    public void periodic() {
        // Constantly push the encoder value to the dashboard so you can tune your limits!
        SmartDashboard.putNumber("Hood Absolute Position", hoodAbsoluteEncoder.get());
    }
}