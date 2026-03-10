package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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
    
    // Motors
    private final SparkMax rightFlywheel = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax leftFlywheel = new SparkMax(19, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(20, MotorType.kBrushless); 

    // Sensors and control
    private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(0);
    private final DigitalInput hoodLimitSwitch = new DigitalInput(1); 
    private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0);

    // State Variables
    private double hoodMinAngle = 0.0; 
    private double hoodMaxAngle = 0.0; 
    private boolean isHoming = false; 
    private boolean isHomed = false; 

    // Interpolating Maps
    private final InterpolatingDoubleTreeMap powerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    
    public ShooterSubsystem() {
        // --- HOOD CONFIG ---
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- FLYWHEEL CONFIG ---
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        // CHANGE THIS TO FALSE IF MOTORS STILL SPIN BACKWARD
        rightConfig.inverted(true); 
        rightConfig.idleMode(IdleMode.kCoast); // Flywheels should coast for better recovery
        rightFlywheel.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        // If motors are mirrored (facing each other), use 'true'. 
        // If they are on the same side, use 'false'.
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

        // Only need to set the leader (right); left follows automatically
        rightFlywheel.set(targetPower);
        setHoodAngle(targetHoodPosition);
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        // We stop the hood motor but keep it homed
        hoodMotor.set(0); 
    }

    public void toggleShooter() {
        if (Math.abs(rightFlywheel.get()) > 0.1) {
            stopShooter();
        } else {
            rightFlywheel.set(0.2); 
        }
    }
    
    public void setHoodAngle(double targetAngle) {
        if (!isHomed) return; 

        // Safety: If limit switch is hit while moving, stop
        if (hoodLimitSwitch.get() && targetAngle < hoodAbsoluteEncoder.get()) {
            hoodMotor.set(0);
            return; 
        }

        double safeTarget = MathUtil.clamp(targetAngle, hoodMinAngle, hoodMaxAngle);
        double currentHoodPosition = hoodAbsoluteEncoder.get();
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
        
        hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.3, 0.3); 
        hoodMotor.set(hoodMotorPower);
    }

    public void setFerryMode() {
        rightFlywheel.set(1.0);
        // Hood to max arc for a long distance "ferry" pass
        setHoodAngle(hoodMaxAngle); 
    }

    @Override
    public void periodic() {
        // --- HOMING LOGIC ---
        if (isHoming && !isHomed) {
            if (!hoodLimitSwitch.get()) {
                hoodMotor.set(-0.2); 
            } else {
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
            }
        }

        // Dashboard Logging
        SmartDashboard.putNumber("FW Output", rightFlywheel.get());
        SmartDashboard.putBoolean("Hood Homed", isHomed);
        SmartDashboard.putBoolean("Switch Pressed", hoodLimitSwitch.get());
    }
}