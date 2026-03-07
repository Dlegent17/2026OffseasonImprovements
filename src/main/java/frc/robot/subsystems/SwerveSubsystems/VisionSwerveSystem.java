package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;

public class VisionSwerveSystem extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final TurretSubsystem turret;
    
    // Make sure this matches the exact name of your Limelight in the web dashboard!
    private final String limelightName = "limelight"; 

    // ==========================================
    // STATIC CAD MEASUREMENTS
    // ==========================================
    
    // 1. Where is the Turret Base relative to the floor center?
    private final Translation3d robotCenterToTurretBase = new Translation3d(0.000, 0.191, 0.310); 

    // 2. Where is the Camera relative to the Turret Base? (When turret is at 0 degrees)
    private final Transform3d turretBaseToCamera = new Transform3d(
        new Translation3d(0.019, -0.118, 0.197), 
        // CHANGED: Roll is 90 for vertical mount! Pitch is -10 (up).
        new Rotation3d(Math.toRadians(90.0), Math.toRadians(-10.0), Math.toRadians(0.0)) 
    );

    public VisionSwerveSystem(SwerveDrivePoseEstimator poseEstimator, TurretSubsystem turret) {
        this.poseEstimator = poseEstimator;
        this.turret = turret;
    }

    /**
     * Calculates the exact 3D position of the Limelight relative to the robot center.
     */
    public Transform3d getDynamicRobotToCamera() {
        double currentTurretAngle = turret.getTurretAngleDegrees();
        
        Rotation3d turretRotation = new Rotation3d(0.0, 0.0, Math.toRadians(currentTurretAngle));
        Pose3d turretPose = new Pose3d(robotCenterToTurretBase, turretRotation);
        Pose3d cameraPose = turretPose.transformBy(turretBaseToCamera);
        
        return new Transform3d(new Pose3d(), cameraPose);
    }

    /**
     * Runs the MegaTag2 pipeline. Call this 50x a second from your Swerve drive periodic!
     * @param currentGyroYawDegrees The live angle of your robot's chassis from the Gyro
     * @param currentGyroRate The rotational velocity of your robot (degrees per second)
     */
    public void updateVision(double currentGyroYawDegrees, double currentGyroRate) {
        
        Transform3d dynamicCameraPos = getDynamicRobotToCamera();

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName, 
            dynamicCameraPos.getX(), 
            dynamicCameraPos.getY(), 
            dynamicCameraPos.getZ(), 
            Math.toDegrees(dynamicCameraPos.getRotation().getX()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getY()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getZ())  
        );

        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            currentGyroYawDegrees, 
            0, 0, 0, 0, 0 
        );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null || mt2.tagCount <= 0) return;
        
        if (Math.abs(currentGyroRate) > 360.0) return; 

        boolean isTrustworthy = (mt2.tagCount >= 2) || (mt2.tagCount == 1 && mt2.avgTagDist < 3.0);

        if (isTrustworthy) {
            poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }

    /**
     * Grabs a pure MegaTag1 vision solve to instantly force-reset the robot's odometry.
     */
    public Pose2d getForceResetPose() {
        Transform3d dynamicCameraPos = getDynamicRobotToCamera();
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName, 
            dynamicCameraPos.getX(), dynamicCameraPos.getY(), dynamicCameraPos.getZ(), 
            Math.toDegrees(dynamicCameraPos.getRotation().getX()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getY()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getZ())
        );

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (mt1 != null && mt1.tagCount > 0) {
            return mt1.pose;
        }
        
        return null; 
    }

    // ==========================================
    // POSE-BASED CHASSIS AIMING
    // ==========================================
    
    private final edu.wpi.first.math.controller.PIDController chassisAimPID = new edu.wpi.first.math.controller.PIDController(2.5, 0.0, 0.0); 
    
    private final edu.wpi.first.math.geometry.Translation2d blueSpeaker = new edu.wpi.first.math.geometry.Translation2d(0.0, 5.55);
    private final edu.wpi.first.math.geometry.Translation2d redSpeaker = new edu.wpi.first.math.geometry.Translation2d(16.54, 5.55);

    public double getAimingRotationSpeed() {
        edu.wpi.first.math.geometry.Pose2d robotPose = poseEstimator.getEstimatedPosition();
        
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        edu.wpi.first.math.geometry.Translation2d target = (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) 
            ? redSpeaker : blueSpeaker;

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        
        double targetAngleRad = Math.atan2(dy, dx);

        chassisAimPID.enableContinuousInput(-Math.PI, Math.PI);
        
        return chassisAimPID.calculate(robotPose.getRotation().getRadians(), targetAngleRad);
    }

    public boolean isAligned() {
        return Math.abs(chassisAimPID.getPositionError()) < 0.035;
    }
    /**
     * Calculates the exact straight-line distance from the robot to the active Alliance Speaker.
     * @return Distance in meters.
     */
    public double getDistanceToSpeakerMeters() {
        // 1. Get where the robot is right now
        Pose2d robotPose = poseEstimator.getEstimatedPosition();
        
        // 2. Pick the correct speaker based on our alliance color
        var alliance = DriverStation.getAlliance();
        Translation2d target = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) 
            ? redSpeaker : blueSpeaker;

        // 3. WPILib calculates the exact hypotenuse (distance) between the two points!
        return robotPose.getTranslation().getDistance(target);
    }
}