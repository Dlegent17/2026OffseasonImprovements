package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.MechanismSubsystems.TurretSubsystem;

public class VisionSwerveSystem extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final TurretSubsystem turret;
    private final String limelightName = "limelight"; 

    private final Translation3d robotCenterToTurretBase = new Translation3d(0.000, 0.191, 0.310); 
    // Get the latest tag ID that Limelight sees
double currentTid = LimelightHelpers.getFiducialID(limelightName);

    // THE FIX: Roll is set to 0.0 because the Limelight Web UI is handling the 90-degree portrait rotation!
// THE FIX: We define the portrait roll AND the backwards yaw here in the code.
// The Web UI offsets will be safely ignored.
private final Transform3d turretBaseToCamera = new Transform3d(
    new Translation3d(-0.12065, 0.118, 0.4064), 
    new Rotation3d(
        Math.toRadians(-90.0),   
        Math.toRadians(0.0),  // PITCH: Your physical camera tilt
        Math.toRadians(165.0)   // YAW: 180 degrees because it faces backwards!
    ) 
);

    public VisionSwerveSystem(SwerveDrivePoseEstimator poseEstimator, TurretSubsystem turret) {
        this.poseEstimator = poseEstimator;
        this.turret = turret;
    }

    public Transform3d getDynamicRobotToCamera() {
        double currentTurretAngle = turret.getTurretAngleDegrees();
        
        Rotation3d turretRotation = new Rotation3d(0.0, 0.0, Math.toRadians(currentTurretAngle));
        Pose3d turretPose = new Pose3d(robotCenterToTurretBase, turretRotation);
        Pose3d cameraPose = turretPose.transformBy(turretBaseToCamera);
        
        return new Transform3d(new Pose3d(), cameraPose);
    }

    public void updateVision(double currentGyroYawDegrees, double currentGyroRate) {
        Transform3d dynamicCameraPos = getDynamicRobotToCamera();

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName, 
            dynamicCameraPos.getX(), dynamicCameraPos.getY(), dynamicCameraPos.getZ(), 
            Math.toDegrees(dynamicCameraPos.getRotation().getX()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getY()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getZ())  
        );

        LimelightHelpers.SetRobotOrientation(limelightName, currentGyroYawDegrees, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null || mt2.tagCount <= 0 || Math.abs(currentGyroRate) > 360.0) return; 

        boolean isTrustworthy = (mt2.tagCount >= 2) || (mt2.tagCount == 1 && mt2.avgTagDist < 3.0);
        if (isTrustworthy) {
            poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }

// Now you can safely use getLockedTX() / getLockedTY() for aiming
    }

    /**
     * Uses MegaTag1 to find the robot's absolute field position.
     * Use this ONLY for initial seeding at the start of a match.
     */
    public Pose2d getForceResetPose() {
        // 1. Calculate where the camera is physically located RIGHT NOW based on turret angle
        Transform3d dynamicCameraPos = getDynamicRobotToCamera();
        
        // 2. Update Limelight's internal 3D offset so it knows the camera moved
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName, 
            dynamicCameraPos.getX(), dynamicCameraPos.getY(), dynamicCameraPos.getZ(), 
            Math.toDegrees(dynamicCameraPos.getRotation().getX()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getY()), 
            Math.toDegrees(dynamicCameraPos.getRotation().getZ())
        );

        // 3. Get the MegaTag1 Pose (Standard 3D geometry)
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // 4. Safety: Only return a pose if we actually see a tag!
        if (mt1 != null && mt1.tagCount > 0) {
            return mt1.pose;
        }
        return null; 
    }
    
    // ==========================================
    // POSE-BASED CHASSIS AIMING (2026 HUB)
    // ==========================================
// 1. INCREASE P SLIGHTLY, AND ADD D-GAIN. 
// The D-gain (0.01) acts as a shock absorber. You may need to tune this up to 0.02 or 0.03.

// Hub Coordinates
private final edu.wpi.first.math.geometry.Translation2d blueHub = new edu.wpi.first.math.geometry.Translation2d(4.03, 4.035); 
private final edu.wpi.first.math.geometry.Translation2d redHub = new edu.wpi.first.math.geometry.Translation2d(12.51, 4.035);

// INCREASED D-GAIN: This acts as a heavy shock absorber. 
// If it still jitters, you can safely raise the D to 0.08 or 0.1
private final PIDController chassisAimPID = new PIDController(0.15, 0.0, 0.1); 

public double getAimingRotationSpeed() {
    if (!LimelightHelpers.getTV(limelightName)) return 0.0;

    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    Translation2d target = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) 
        ? redHub : blueHub;

    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();
    double targetAngleRad = Math.atan2(dy, dx);
    
    chassisAimPID.enableContinuousInput(-Math.PI, Math.PI);
    
    // 1. Calculate speed to update the internal error
    double speed = chassisAimPID.calculate(robotPose.getRotation().getRadians(), targetAngleRad);
    double currentError = chassisAimPID.getPositionError();

    // 2. WIDER DEADBAND (~3 Degrees)
    // If we are inside this window, cut power to 0 immediately.
    if (Math.abs(currentError) < 0.05) {
        return 0.0;
    }

    // 3. THE SMART KICK
    // Only apply the minimum friction kick if we are more than ~8 degrees away.
    // This prevents the kick from punching the robot completely through the deadband!
    double minSpeed = 0.08; 
    if (Math.abs(currentError) > 0.15) {
        speed += Math.copySign(minSpeed, speed);
    }

    // 4. Invert and Clamp
    return Math.max(-1.0, Math.min(1.0, -speed));
}

    public boolean isAligned() {
    // Returns true if the robot is within ~3 degrees of the target.
    // I matched this 0.05 number to the deadband we just added!
    return Math.abs(chassisAimPID.getPositionError()) < 0.05;
}
    public double getDistanceToHubMeters() {
        Pose2d robotPose = poseEstimator.getEstimatedPosition();
        var alliance = DriverStation.getAlliance();
        Translation2d target = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) 
            ? redHub : blueHub;

        return robotPose.getTranslation().getDistance(target);
    }

@Override
    public void periodic() {
        // Broadcast the live distance to the dashboard so you can write it in your notebook
        SmartDashboard.putNumber("LIVE Distance (Meters)", getDistanceToHubMeters());
    }
}
