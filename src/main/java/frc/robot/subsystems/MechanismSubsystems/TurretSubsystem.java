package frc.robot.subsystems.MechanismSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor;
    
    // REPLACE 100.0 WITH YOUR ACTUAL TURRET GEAR RATIO
    // Example: If it takes 100 motor spins for 1 turret spin, ratio is 100.0
    private final double TURRET_GEAR_RATIO = 100.0; 

    public TurretSubsystem() {
        turretMotor = new SparkMax(24, MotorType.kBrushless);
        SparkMaxConfig turretConfig = new SparkMaxConfig();
        
        turretConfig.smartCurrentLimit(30); 
        turretConfig.idleMode(IdleMode.kBrake); 
        
        turretConfig.softLimit.forwardSoftLimitEnabled(true);
        turretConfig.softLimit.forwardSoftLimit(1.5); 
        turretConfig.softLimit.reverseSoftLimitEnabled(true);
        turretConfig.softLimit.reverseSoftLimit(-1.5); 

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   public void setTurretSpeed(double speed) {
    double currentAngle = getTurretAngleDegrees();

    // With -speed at the bottom:
    // A positive speed input actually makes the motor go NEGATIVE (Left).
    // A negative speed input actually makes the motor go POSITIVE (Right).

  /*   if (currentAngle > 300 && speed < 0) { 
        // We are at the Right limit. If speed is negative, the motor 
        // would try to go further Right. Block it.
        turretMotor.set(0); 
    } 
    else if (currentAngle < -300 && speed > 0) { 
        // We are at the Left limit. If speed is positive, the motor 
        // would try to go further Left. Block it.
        turretMotor.set(0); 
    } 
    else {
        // Safe zone: Apply the inverted speed
        turretMotor.set(-speed); 
    } */
}
/**
     * A Command that manually rotates the turret to the right.
     */
    public Command turnRightCommand() {
        return this.runEnd(() -> setTurretSpeed(-0.1), () -> setTurretSpeed(0.0));
    }

    /**
     * A Command that manually rotates the turret to the left.
     */
    public Command turnLeftCommand() {
        return this.runEnd(() -> setTurretSpeed(0.1), () -> setTurretSpeed(0.0));
    }
    /**
     * Converts raw motor rotations into actual physical turret degrees.
     * This is CRITICAL for MegaTag2 to know where the camera is pointing.
     */
    public double getTurretAngleDegrees() {
        double motorRotations = turretMotor.getEncoder().getPosition();
        return (motorRotations / TURRET_GEAR_RATIO) * 360.0; 
    }
    @Override
public void periodic() {
   double ty = LimelightHelpers.getTY("limelight");
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    // This sends it to the RioLog (The text list you see now)
    System.out.println("Current TY: " + ty);

    // This sends it to the Dashboard (The visual gauges)
    SmartDashboard.putNumber("Limelight TY", ty);
    
    // This creates a "Centered" indicator
    boolean isCentered = hasTarget && Math.abs(ty) < 1.0;
    SmartDashboard.putBoolean("Target Centered", isCentered);
}
}