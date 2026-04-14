package frc.robot.subsystems.MechanismSubsystems;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor;

    // The gear ratio of the turret. This is how many times the motor rotates for one full rotation of the turret.
    private final double TURRET_GEAR_RATIO = 50.0;
    // Limits for the turret in degrees. These are the physical limits of how far the turret can rotate in either direction.
    private static final double FORWARD_LIMIT_DEGREES =90.0; // max rightward rotation
    private static final double REVERSE_LIMIT_DEGREES = -90.0; // max leftward rotation

    @SuppressWarnings("removal")
	public TurretSubsystem() {
        turretMotor = new SparkMax(24, MotorType.kBrushless);
        SparkMaxConfig turretConfig = new SparkMaxConfig();

        turretConfig.smartCurrentLimit(30);
        turretConfig.idleMode(IdleMode.kBrake);

        // Convert degree limits to motor rotations for the hardware soft limits
        turretConfig.softLimit.forwardSoftLimitEnabled(true);
        turretConfig.softLimit.forwardSoftLimit((float) degreesToMotorRotations(FORWARD_LIMIT_DEGREES));
        turretConfig.softLimit.reverseSoftLimitEnabled(true);
        turretConfig.softLimit.reverseSoftLimit((float) degreesToMotorRotations(REVERSE_LIMIT_DEGREES));

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private double degreesToMotorRotations(double degrees) {
        return (degrees / 360.0) * TURRET_GEAR_RATIO;
}

    public void setTurretSpeed(double speed) {
        double currentAngle = getTurretAngleDegrees();

        //SmartDashboard.putNumber("Turret Realtime Angle", currentAngle);

        // Software limits using the same degree constants as the hardware limits.
        // These act as a secondary safety layer on top of the SparkMax soft limits.
        if (currentAngle > FORWARD_LIMIT_DEGREES && speed < 0) {
            turretMotor.set(0);
        } else if (currentAngle < REVERSE_LIMIT_DEGREES && speed > 0) {
            turretMotor.set(0);
        } else {
            turretMotor.set(-speed);
        }
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
    }
}
