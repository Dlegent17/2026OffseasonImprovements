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
    
    // --- MOT
    private final SparkMax topFlywheel = new SparkMax(30, MotorType.kBrushless);
    private final SparkMax bottomFlywheel = new SparkMax(31, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(32, MotorType.kBrushless); 

    // --- HOOD SENSOR & CONTROL ---
    // Assuming the absolute encoder is plugged into DIO Port 0 on the RoboRIO
    private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(0);
    private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0);

    // --- SAFETY LIMITS (TUNE THESE!) ---
    // These represent the physical min and max rotations of your encoder
    private final double HOOD_MIN_ANGLE = 0.10; // Bottom hard stop
    private final double HOOD_MAX_ANGLE = 0.45; // Top hard stop

    // The Interpolating Map - Tells the motors how hard to shoot depending on the position relative to April Tag
    private final InterpolatingDoubleTreeMap powerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
@SuppressWarnings("removal")
    public ShooterSubsystem() {

        // --- NEW REVLIB MOTOR CONFIGURATION ---
        // Make the hood motor brake so it doesn't fall down when disabled
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // 1. FLYWHEEL POWER (Percentage 0.0 to 1.0)
        powerMap.put(1.5, 0.40); 
        powerMap.put(3.0, 0.65); 
        powerMap.put(5.0, 0.90); 

        // 2. HOOD ANGLE (Absolute Encoder Position)
        hoodMap.put(1.5, 0.12);  // Close shot: Hood mostly down
        hoodMap.put(3.0, 0.25);  // Mid shot: Hood half up
        hoodMap.put(5.0, 0.40);  // Far shot: Hood fully raised
    }

    /**
     * Looks at the distance, checks the maps, and automatically adjusts the flywheels and hood!
     */
    public void setDynamicShooter(double distanceToHubMeters) {
        // 1. Get the magic numbers from the maps
        double targetPower = powerMap.get(distanceToHubMeters);
        double targetHoodPosition = hoodMap.get(distanceToHubMeters);

        // 2. Clamp the requested hood position so it NEVER exceeds your mechanical limits
        targetHoodPosition = MathUtil.clamp(targetHoodPosition, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        // 3. Apply Power to Flywheels
        topFlywheel.set(targetPower);
        bottomFlywheel.set(targetPower);

        // 4. Calculate PID for the Hood and apply motor power (Using .get() instead of .getAbsolutePosition())
        double currentHoodPosition = hoodAbsoluteEncoder.get();
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, targetHoodPosition);
        
        // Limit the max speed of the hood so it doesn't snap violently
        hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.4, 0.4); 
        hoodMotor.set(hoodMotorPower);
    }

    /** Simple toggle for testing flywheels manually */
    public void toggleShooter() {
        if (topFlywheel.get() > 0.1) { stopShooter(); } 
        else { topFlywheel.set(0.5); bottomFlywheel.set(0.5); }
    }

    public void stopShooter() {
        topFlywheel.set(0);
        bottomFlywheel.set(0);
        hoodMotor.set(0); // PID stops running, brake mode holds it in place
    }

    @Override
    public void periodic() {
        // Constantly push the encoder value to the dashboard so you can tune your limits!
        SmartDashboard.putNumber("Hood Absolute Position", hoodAbsoluteEncoder.get());
    }
}