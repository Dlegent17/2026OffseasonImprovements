package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
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

    // Defining Hood sensors and control
    private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(0);
    private final DigitalInput hoodLimitSwitch = new DigitalInput(1); // Limit Switch on DIO 1
    private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0);

    // Dynamic Safety Limits & State
    private double hoodMinAngle = 0.0; 
    private double hoodMaxAngle = 0.0; 
    
    // --- NEW: Tracking variables for the homing routine ---
    private boolean isHoming = false; 
    private boolean isHomed = false; 

    // The Interpolating Maps
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

        // We DO NOT set the hoodMap here anymore! It generates automatically when homed.
    }

    // ==========================================
    // THE MISSING METHOD!
    // ==========================================
    
    /** Triggered by Robot.java on start to begin the homing sequence */
    public void startHoming() {
        // Force the homing routine to start
        isHoming = true;
        
        // Tell the code to "forget" the old zero so it locks out the shooter 
        // until the limit switch is hit again
        isHomed = false; 
    }

    // ==========================================
    // SHOOTER MODES
    // ==========================================
    
    public void setDynamicShooter(double distanceToHubMeters) {
        if (!isHomed) return; // Prevent firing if not zeroed

        double targetPower = powerMap.get(distanceToHubMeters);
        double targetHoodPosition = hoodMap.get(distanceToHubMeters);

        rightFlywheel.set(targetPower);
        leftFlywheel.set(targetPower);
        setHoodAngle(targetHoodPosition);
    }

    public void setFerryMode() {
        if (!isHomed) return; 

        rightFlywheel.set(1.0);
        leftFlywheel.set(1.0);
        setHoodAngle(hoodMaxAngle); 
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        leftFlywheel.set(0);
        
        // Only force stop the hood if we aren't currently trying to home it!
        if (isHomed) {
            hoodMotor.set(0); 
        }
    }

    public void toggleShooter() {
        if (rightFlywheel.get() > 0.1) { stopShooter(); } 
        else { rightFlywheel.set(-0.1); leftFlywheel.set(0.1); }
    }
    
    public Command toggleShooterCommand() {
        return this.runOnce(() -> toggleShooter());
    }

    // ==========================================
    // HOOD CONTROL HELPER
    // ==========================================

    public void setHoodAngle(double targetAngle) {
        if (!isHomed) return; // Do nothing if not homed

        // HARDWARE LIMIT SWITCH OVERRIDE
        if (hoodLimitSwitch.get()) {
            hoodMotor.set(0);
            return; 
        }

        // Clamp & Calculate PID
        double safeTarget = MathUtil.clamp(targetAngle, hoodMinAngle, hoodMaxAngle);
        double currentHoodPosition = hoodAbsoluteEncoder.get();
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
        
        hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.1, 0.1); 
        hoodMotor.set(hoodMotorPower);
    }

    @Override
    public void periodic() {
        
        // ==========================================
        // HOMING ROUTINE EXECUTION
        // ==========================================
        if (isHoming && !isHomed) {
            // Drive down until we hit the switch
            if (!hoodLimitSwitch.get()) {
                hoodMotor.set(-0.15); // WARNING: Ensure negative moves DOWN!
            } else {
                // We hit the switch! Stop the motor.
                hoodMotor.set(0);

                // 1. Get position and round it
                double rawStartPos = hoodAbsoluteEncoder.get();
                hoodMinAngle = Math.round(rawStartPos * 100.0) / 100.0;
                
                // 2. Set max limit
                hoodMaxAngle = hoodMinAngle + 0.38;

                // 3. Dynamically set angle maps
                hoodMap.clear();
                hoodMap.put(1.5, hoodMinAngle + 0.02);
                hoodMap.put(3.0, hoodMinAngle + 0.15);
                hoodMap.put(5.0, hoodMinAngle + 0.30);

                // 4. Lock it in so we can shoot!
                isHomed = true;
                isHoming = false;
                System.out.println("Hood Homed! Min: " + hoodMinAngle);
            }
        }

        // ==========================================
        // DASHBOARD UPDATES
        // ==========================================
        SmartDashboard.putNumber("Hood Absolute Position", hoodAbsoluteEncoder.get());
        SmartDashboard.putBoolean("Hood Homed", isHomed);
        SmartDashboard.putBoolean("Hood Limit Hit", hoodLimitSwitch.get());
        if (isHomed) {
            SmartDashboard.putNumber("Dynamic Hood Min", hoodMinAngle);
            SmartDashboard.putNumber("Dynamic Hood Max", hoodMaxAngle);
        }
    }
}