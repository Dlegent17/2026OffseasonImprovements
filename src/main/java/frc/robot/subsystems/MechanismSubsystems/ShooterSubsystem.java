package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
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
    // Constants
	private static final int right_flywheel_id = 18;
	private static final int left_flywheel_id = 19;
	private static final int hood_motor_id = 20;
	private static final int hood_limit_switch_port = 1;
	private static final double hood_HOMIMG_SPEED = -0.2; 
    private static final double HOOD_BOTTOMING_SPEED = -0.05; 
    private static final double HOOD_MAX_TRAVEL_ROTATIONS = 57.0;
    // Hardware For Shooter Subsystem
    private final SparkMax rightFlywheel = new SparkMax(right_flywheel_id, MotorType.kBrushless);
    private final SparkMax leftFlywheel = new SparkMax(left_flywheel_id, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(hood_motor_id, MotorType.kBrushless); 
    private final RelativeEncoder hoodRelativeEncoder = hoodMotor.getEncoder();
    private final DigitalInput hoodLimitSwitch = new DigitalInput(hood_limit_switch_port); 
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
		configureMotors();
		
		//Initialize Motor Power Map
        powerMap.put(2.3, 0.8); 
        powerMap.put(3.0, 0.9); 
        powerMap.put(5.0, 1.0);
        
        startHoming();
	}
    private void configureMotors () {
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

	}
	//Commands and Public Methods
	    public void startHoming() {
        isHoming = true;
        isHomed = false; 
    }
    public void stopShooterAndStartHoming() {
        rightFlywheel.set(0);
            startHoming();           
    }

    public void setDynamicShooter(double distanceToHubMeters) {
        if (!isHomed) return;       
        currentHoodTarget = hoodMap.get(distanceToHubMeters);
        rightFlywheel.set(powerMap.get(distanceToHubMeters));
		// Add in again if we are using this variable for shuffleboard
        // double targetPower = powerMap.get(distanceToHubMeters);  
		//If you need targetpower comment out 2nd command above and add this: 
		//rightFlywheel.set(targetPower)
    }

    public void runFixedShooter() {
        rightFlywheel.set(1.0); //TODO: Tune This
        currentHoodTarget = hoodMinAngle;
    }
    public void runPassingShooter() {
        rightFlywheel.set(0.75); //TODO: Tune This
        currentHoodTarget = hoodMinAngle + 50;
    }

    public Command fixedShooter() {
        return this.runEnd(this::runFixedShooter, this::stopFixedShooter);
    }
    public Command passingShooter() {
        return this.runEnd(this::runPassingShooter, this::stopFixedShooter);
    }

    public void stopFixedShooter() {
        rightFlywheel.set(0.0);
    }

    public void stopShooter() {
        rightFlywheel.set(0);
        
        // This acts as a "Return to Zero" state.
        if (isHomed) {
            currentHoodTarget = hoodMinAngle; 
        }
    }

    // public void toggleShooter() {
    //     if (Math.abs(rightFlywheel.get()) > 0.1) {
    //         stopShooter();
    //     } else {
    //         rightFlywheel.set(0.2); 
    //     }
    // }
    
    public void setHoodAngle(double targetAngle) {
        if (!isHomed) return; 
        currentHoodTarget = targetAngle;
    }
	//Perodic Logic
@Override
    public void periodic() {
        if (isHoming && !isHomed) {
            handleHomingSequence();
        } else if (isHomed && !isHoming) {
            handleHoodPIDControl();
        }
        
        updateTelemetry();
    }

    private void handleHomingSequence() {
        if (!hoodLimitSwitch.get()) {
            hoodMotor.set(hood_HOMIMG_SPEED); 
        } else {
            hoodMotor.set(0);

            double rawStartPos = hoodRelativeEncoder.getPosition();
            hoodMinAngle = Math.round(rawStartPos * 100.0) / 100.0;
            hoodMaxAngle = hoodMinAngle + HOOD_MAX_TRAVEL_ROTATIONS; 

            populateHoodMap();
            currentHoodTarget = hoodMinAngle;

            isHomed = true;
            isHoming = false;
        }
    }

    private void populateHoodMap() {
        hoodMap.clear();
        hoodMap.put(2.3, hoodMinAngle + 0);  // Close shot
        hoodMap.put(3.0, hoodMinAngle + 30); // Mid shot
        hoodMap.put(5.0, hoodMinAngle + 40); // Far shot
    }

    private void handleHoodPIDControl() {
        double currentHoodPosition = hoodRelativeEncoder.getPosition();
        
        // Push gently against the bottom limit switch if targeted at min angle
        if (currentHoodTarget <= hoodMinAngle + 0.01) {
            hoodMotor.set(!hoodLimitSwitch.get() ? HOOD_BOTTOMING_SPEED : 0.0);
            return;
        } 
        
        // Otherwise, run normal PID control
        double safeTarget = MathUtil.clamp(currentHoodTarget, hoodMinAngle, hoodMaxAngle);
        double hoodMotorPower = hoodPID.calculate(currentHoodPosition, safeTarget);
        
        // Safety: Prevent driving down if the limit switch is already pressed
        if (hoodLimitSwitch.get() && hoodMotorPower < 0) {
            hoodMotor.set(0);
        } else {
            hoodMotor.set(MathUtil.clamp(hoodMotorPower, -0.3, 0.3)); 
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("FW Output", rightFlywheel.get());
        SmartDashboard.putBoolean("Switch Pressed", hoodLimitSwitch.get());
        SmartDashboard.putNumber("rawStartPos", hoodRelativeEncoder.getPosition());
    }
}
