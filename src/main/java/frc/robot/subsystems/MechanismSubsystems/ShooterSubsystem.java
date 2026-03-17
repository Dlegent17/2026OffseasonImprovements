package frc.robot.subsystems.MechanismSubsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.LimelightHelpers;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
    
    // Motors for the flywheel and hood. The flywheel is a simple master-follower setup, while the hood has its own motor and encoder for precise control.
    private final SparkMax rightFlywheel = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax leftFlywheel = new SparkMax(19, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(20, MotorType.kBrushless); 
    private final RelativeEncoder hoodRelativeEncoder = hoodMotor.getEncoder();

    // Sensors and control components for the hood. We are using a relative encoder on the motor and a limit switch at the bottom to establish our zero point.
    //private final DutyCycleEncoder hoodAbsoluteEncoder = new DutyCycleEncoder(0);
    private final DigitalInput hoodLimitSwitch = new DigitalInput(1); 
    private final PIDController hoodPID = new PIDController(0.1, 0.0, 0.0);

    // State Variables for Homing and Dynamic Control
    private double hoodMinAngle = 0.0; 
    private double hoodMaxAngle = 0.0; 
    private boolean isHoming = false; 
    private boolean isHomed = false; 
    private double currentHoodTarget = 0.0; 

    // Interpolating Maps for dynamic adjustments based on distance to the target. These will be populated after homing to ensure we have accurate angle limits.
    private final InterpolatingDoubleTreeMap powerMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    
    @SuppressWarnings("removal")
	public ShooterSubsystem() {
        // Hood Configuration: Brake Mode and Current Limit
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Wheel Configuration: Inverted and Coast Mode
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.inverted(true); 
        rightConfig.idleMode(IdleMode.kCoast); 
        rightFlywheel.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.follow(rightFlywheel, true); 
        leftFlywheel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Interpolating Maps: Distance (m) to Power and Hood Angle
        powerMap.clear();
        powerMap.put(2.3, 1.0); 
        // powerMap.put(3.0, 0.85); 
        // powerMap.put(5.0, 1.00);
        
        startHoming();
    }

    public void stopShooterAndStartHoming() {
        rightFlywheel.set(0);
            startHoming();
                
    }

    public void startHoming() {
        isHoming = true;
        isHomed = false; 
    }

    public void setDynamicShooter(double distanceToHubMeters) {
        if (!isHomed) return; 

        double targetPower = powerMap.get(distanceToHubMeters);
        
        // Just update the target! periodic() handles the movement.
        currentHoodTarget = hoodMap.get(distanceToHubMeters);
        rightFlywheel.set(targetPower);
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        
        // This acts as a "Return to Zero" state.
        if (isHomed) {
            currentHoodTarget = hoodMinAngle; 
        }
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
        // Manually override the target if needed
        currentHoodTarget = targetAngle;
    }

    public void setFerryMode() {
        rightFlywheel.set(1.0);
        currentHoodTarget = (hoodMaxAngle - 10);
        // Homing Logic: This runs first to establish the zero point and limits before any PID control takes over.
        if (isHoming && !isHomed) {
            if (!hoodLimitSwitch.get()) {
                hoodMotor.set(-0.2); 
            } else {
                hoodMotor.set(0);

                double rawStartPos = hoodRelativeEncoder.getPosition();
                //hoodRelativeEncoder.setPosition(rawStartPos);
                hoodMinAngle = Math.round(rawStartPos * 100.0) / 100.0;
                hoodMaxAngle = hoodMinAngle + 57.57; //Max angle is 57.57 motor rotations above the min angle, which we found through testing. This is how far the hood can actually move up before hitting the physical stop.
                hoodMap.clear();
                hoodMap.put(2.3, hoodMinAngle + 0); // Close shot: Just slightly pitched up
                // hoodMap.put(3.0, hoodMinAngle + 30); // Mid shot: Halfway up the 1.5 range
                // hoodMap.put(5.0, hoodMinAngle + 40); // Far shot: High angle arc
                
                // Immediately set the default state to resting at the bottom
                currentHoodTarget = hoodMinAngle;

                isHomed = true;
                isHoming = false;
            }
        }

        // PID Control Logic: This runs after homing is complete to maintain the hood angle and adjust flywheel power based on distance.
        // This is what actually forces the motor to move to your targett angle and keeps it there, even if the robot is shaking or the battery is low or whatever.
        if (isHomed && !isHoming) {
            double currentHoodPosition = hoodRelativeEncoder.getPosition();
            
            // If the target is the absolute bottom, gently push until the switch clicks
            if (currentHoodTarget <= hoodMinAngle + 0.01) {
                if (!hoodLimitSwitch.get()) {
                    hoodMotor.set(-0.15); 
                } else {
                    hoodMotor.set(0); 
                }
            } 
            // Otherwise, use the PID to dynamically track the target angle
            else {
                double safeTarget = MathUtil.clamp(currentHoodTarget, hoodMinAngle, hoodMaxAngle);
                double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
                
                // Safety: Stop if switch is pressed while moving down
                if (hoodLimitSwitch.get() && hoodMotorPower < 0) {
                    hoodMotor.set(0);
                } else {
                    hoodMotorPower = MathUtil.clamp(hoodMotorPower, -0.3, 0.3); 
                    hoodMotor.set(hoodMotorPower);
                }
            }
        }

        // Dashboard Logging
        SmartDashboard.putNumber("FW Output", rightFlywheel.get());
        // SmartDashboard.putBoolean("Hood Homed", isHomed);
        // SmartDashboard.putBoolean("Switch Pressed", hoodLimitSwitch.get());
       // SmartDashboard.putNumber("Hood Target", currentHoodTarget);
        // SmartDashboard.putNumber("Hood Current", hoodAbsoluteEncoder.get());
        SmartDashboard.putNumber("rawStartPos", hoodRelativeEncoder.getPosition());
    }
}