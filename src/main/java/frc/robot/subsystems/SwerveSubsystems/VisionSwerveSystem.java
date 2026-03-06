package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers; // Make sure LimelightHelpers.java is in your frc.robot folder!
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSwerveSystem extends SubsystemBase {

    private final String llName = "limelight"; 
    private final SwerveDrivePoseEstimator poseEstimator; 
    private final PIDController aimController;
    private final TurretSubsystem turret;

    // 1. Where is the Turret Base relative to the floor center?
    private final Translation3d robotCenterToTurretBase = new Translation3d(0.000, 0.191, 0.310); 

    // 2. Where is the Camera relative to the Turret Base? 
    // Pitch is -10 degrees (tilted up), Yaw is 90 degrees (facing left)
    private final Transform3d turretBaseToCamera = new Transform3d(
        new Translation3d(0.019, -0.118, 0.197), 
        new Rotation3d(0.0, Math.toRadians(-10.0), Math.toRadians(90.0)) 
    );
    /**
     * Constructor
     */
    public VisionSwerveSystem(SwerveDrivePoseEstimator poseEstimator, TurretSubsystem turret) {
        this.poseEstimator = poseEstimator;
        this.turret = turret; // <--- THIS IS THE MAGIC LINE! Saves it so the math works.
        
        // Initialize PID Controller (kP, kI, kD) - Start with these, tune later
        this.aimController = new PIDController(0.04, 0.0, 0.002);
        this.aimController.setTolerance(1.5); 
    }
/**
     * Calculates the exact 3D position of the Limelight relative to the robot center, 
     * accounting for the live rotation of the turret.
     */
    public Transform3d getDynamicRobotToCamera() {
        // Read the live angle from the turret subsystem
        double currentTurretAngle = turret.getTurretAngleDegrees();
        
        // Rotate the base, apply the offset, and return the new position
        Rotation3d turretRotation = new Rotation3d(0.0, 0.0, Math.toRadians(currentTurretAngle));
        Pose3d turretPose = new Pose3d(robotCenterToTurretBase, turretRotation);
        Pose3d cameraPose = turretPose.transformBy(turretBaseToCamera);
        
        return new Transform3d(new Pose3d(), cameraPose);
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

        // 2.5 Calculate the camera's live position and send it to the Limelight
        Transform3d dynamicCamera = getDynamicRobotToCamera();
        
        LimelightHelpers.setCameraPose_RobotSpace(
            llName, 
            dynamicCamera.getTranslation().getX(), // Forward/Back
            dynamicCamera.getTranslation().getY(), // Left/Right
            dynamicCamera.getTranslation().getZ(), // Up/Down
            Math.toDegrees(dynamicCamera.getRotation().getX()), // Roll
            Math.toDegrees(dynamicCamera.getRotation().getY()), // Pitch
            Math.toDegrees(dynamicCamera.getRotation().getZ())  // Yaw
        );
        // ---------------------------

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
    /**
     * Fetches a pure vision pose (MegaTag 1) for a hard odometry reset.
     * We use MegaTag 1 here instead of MegaTag 2 because if the robot is completely lost,
     * its gyro is probably wrong too, and MegaTag 2 relies on the gyro.
     * * @return The 2D Pose from vision, or null if no tags are visible.
     */
    public Pose2d getForceResetPose() {
        // Use the standard MegaTag 1 estimate (wpiBlue)
        LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
        
        // Only return it if we actually see a tag
        if (mt1Pose.tagCount > 0) {
            return mt1Pose.pose;
        }
        
        // Return null if we are blind
        return null;
    }
}