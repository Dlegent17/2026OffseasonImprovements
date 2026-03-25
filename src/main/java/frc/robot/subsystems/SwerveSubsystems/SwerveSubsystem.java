package frc.robot.subsystems.SwerveSubsystems;
import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import com.pathplanner.lib.config.RobotConfig;         
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
public class SwerveSubsystem extends SubsystemBase
{
private final SwerveDrive swerveDrive;
private final Field2d field2d = new Field2d();
   public SwerveSubsystem(File directory)
  { 
    SmartDashboard.putData("Field", field2d);
    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // Set the verbosity of the telemetry.  This will affect how much data is sent to the dashboard and how often.  Adjust this based on your needs and the performance of your dashboard.
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.

  }
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }
  @Override
  public void periodic()
  {
    field2d.setRobotPose(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
  }
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds)
  {
    swerveDrive.addVisionMeasurement(visionPose, timestampSeconds);
  }
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); 
  }
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }
  public void lock()
  {
    swerveDrive.lockPose();
  }
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
public double getTurnRate()
{
    return Math.toDegrees(getSwerveDrive().getFieldVelocity().omegaRadiansPerSecond);
}
public edu.wpi.first.math.estimator.SwerveDrivePoseEstimator getPoseEstimator() {
      return swerveDrive.swerveDrivePoseEstimator;
  }
 public void setupPathPlanner() {
    RobotConfig config;
    try {      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
        this::getPose, 
        this::resetOdometry, 
        this::getRobotVelocity, 
        this::setChassisSpeeds, 
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID
            new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
        ),
        config,
        () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this
    );
}
}
