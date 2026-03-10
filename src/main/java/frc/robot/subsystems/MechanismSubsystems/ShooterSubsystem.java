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
    private final DigitalInput hoodLimitSwitch = new DigitalInput(1); 
    private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0);

    // State Variables
    private double hoodMinAngle = 0.0; 
    private double hoodMaxAngle = 0.0; 
    private boolean isHoming = false; 
    private boolean isHomed = false; 

    // The Interpolating Maps
    private final InterpolatingDoubleTreeMap powerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    
    public ShooterSubsystem() {
        // --- HOOD CONFIG ---
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- FLYWHEEL SYNC (The "Handcuff" Fix) ---
        // This tells the left motor: "Do exactly what the right motor does, but inverted."
        // This prevents them from fighting each other and getting hot!
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(rightFlywheel, true); 
        leftFlywheel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- INTERPOLATION MAPS ---
        powerMap.put(1.5, 0.45); 
        powerMap.put(3.0, 0.75); 
        powerMap.put(5.0, 1.00); 
    }

    public void startHoming() {
        isHoming = true;
        isHomed = false; 
    }

    public void setDynamicShooter(double distanceToHubMeters) {
        if (!isHomed) return; 

        double targetPower = powerMap.get(distanceToHubMeters);
        double targetHoodPosition = hoodMap.get(distanceToHubMeters);

        // Only set Right; Left follows automatically now
        rightFlywheel.set(targetPower);
        setHoodAngle(targetHoodPosition);
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        if (isHomed) {
            hoodMotor.set(0); 
        }
    }

    public void toggleShooter() {
        if (Math.abs(rightFlywheel.get()) > 0.1) {
            stopShooter();
        } else {
            // Setting the leader (right) automatically sets the follower (left)
            rightFlywheel.set(0.2); 
        }
    }
    
    public void setHoodAngle(double targetAngle) {
        if (!isHomed) return; 

        // SAFETY: TRUE = Pressed. If pressed, stop!
        if (hoodLimitSwitch.get()) {
            hoodMotor.set(0);
            return; 
        }

        double safeTarget = MathUtil.clamp(targetAngle, hoodMinAngle, hoodMaxAngle);
        double currentHoodPosition = hoodAbsoluteEncoder.get();
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
        
        // Increased power: 0.1 was too weak to move the heavy hood
        hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.3, 0.3); 
        hoodMotor.set(hoodMotorPower);
    }

    @Override
    public void periodic() {
        // --- HOMING LOGIC ---
        if (isHoming && !isHomed) {
            // If NOT pressed (false), keep driving down
            if (!hoodLimitSwitch.get()) {
                hoodMotor.set(-0.2); 
            } else {
                // If Pressed (true), STOP and lock in home
                hoodMotor.set(0);

                double rawStartPos = hoodAbsoluteEncoder.get();
                hoodMinAngle = Math.round(rawStartPos * 100.0) / 100.0;
                hoodMaxAngle = hoodMinAngle + 0.38;

                hoodMap.clear();
                hoodMap.put(1.5, hoodMinAngle + 0.02);
                hoodMap.put(3.0, hoodMinAngle + 0.15);
                hoodMap.put(5.0, hoodMinAngle + 0.30);

                isHomed = true;
                isHoming = false;
                System.out.println("Homing Complete!");
            }
        }

        // --- DASHBOARD ---
        SmartDashboard.putNumber("FW Output", rightFlywheel.get());
        SmartDashboard.putBoolean("Hood Homed", isHomed);
        SmartDashboard.putBoolean("Switch Pressed", hoodLimitSwitch.get());
    }
}