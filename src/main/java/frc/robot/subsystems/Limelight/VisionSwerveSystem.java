package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator; 
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers; // Make sure LimelightHelpers.java is in your frc.robot folder!

public class VisionSwerveSystem extends SubsystemBase {

    private final String llName = "limelight"; 
    private final SwerveDrivePoseEstimator poseEstimator; 
    private final PIDController aimController;

    /**
     * Constructor
     */
    public VisionSwerveSystem(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
        
        // Initialize PID Controller (kP, kI, kD) - Start with these, tune later
        this.aimController = new PIDController(0.04, 0.0, 0.002);
        this.aimController.setTolerance(1.5); 
    }

    /**
     * Runs 50 times a second (every 20ms) in the background.
     */
    @Override
    public void periodic() {
        // 1. Get the current gyro heading from your pose estimator
        Rotation2d gyroHeading = poseEstimator.getEstimatedPosition().getRotation();

        // 2. Feed the gyro heading to the Limelight (CRITICAL for MegaTag 2)
        LimelightHelpers.SetRobotOrientation(llName, 
            gyroHeading.getDegrees(), 
            0, 0, 0, 0, 0);

        // 3. Fetch the MegaTag 2 Pose Estimate
        LimelightHelpers.PoseEstimate mt2Pose = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

        // 4. Filter and apply the vision measurement
        if (mt2Pose.tagCount > 0 && mt2Pose.avgTagDist < 4.0) {
            poseEstimator.addVisionMeasurement(
                mt2Pose.pose, 
                mt2Pose.timestampSeconds
            );
        }

        // Dashboard Debugging
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putBoolean("Vision/Is Aligned", isAligned());
        SmartDashboard.putNumber("Vision/TX (Error)", LimelightHelpers.getTX(llName));
    }

    /**
     * Calculates the rotation speed needed to center the Limelight on the tag.
     */
    public double getAimingRotationSpeed() {
        if (hasTarget()) {
            double tx = LimelightHelpers.getTX(llName);
            return aimController.calculate(tx, 0.0);
        }
        return 0.0;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(llName);
    }

    public boolean isAligned() {
        return hasTarget() && aimController.atSetpoint();
    }
}