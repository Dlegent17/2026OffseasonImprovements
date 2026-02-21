// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// REV Imports
/* import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode; */


public class Robot extends TimedRobot
{
  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  // --- VARIABLES FOR MOTOR TEST ---
  //private SparkMax testMotor;
  //private XboxController driverController; 
  // --------------------------------

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()////
  {
    return instance;
  }

  @Override
  public void robotInit()
  {
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();
    // --- MOTOR SETUP ---
    // 1. Initialize the motor on CAN ID 43
    //testMotor = new SparkMax(99, MotorType.kBrushless);

    // 2. Configure Current Limit (Safe for Neo 550)
    //SparkMaxConfig config = new SparkMaxConfig();
    //config.smartCurrentLimit(25);
    
    // Apply config (Ignore yellow warnings, they are fine for now)
    //testMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // 3. Initialize the Controller on Port 0
    //driverController = new XboxController(0);
    // -------------------

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
// Get robot's rotational speed
    double omegaRps = m_robotContainer.getDrivebase().getSwerveDrive().getFieldVelocity().omegaRadiansPerSecond;

    // Get Limelight data
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    // Initial check: Do we have data, and is the robot stable enough (not spinning wildly)?
    if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        
        // 1. Get the average distance to the tags we currently see
        double averageTagDistance = llMeasurement.avgTagDist;

        // 2. Determine if we should trust this measurement
        boolean isTrustworthy = false;

        if (llMeasurement.tagCount >= 2) {
            // MULTI-TAG: Very accurate. We can trust this from far away.
            if (averageTagDistance < 4.5) {
                isTrustworthy = true;
            }
        } else if (llMeasurement.tagCount == 1) {
            // SINGLE-TAG: Prone to noise. Only trust it when we are close.
            if (averageTagDistance < 2.5) {
                isTrustworthy = true;
            }
        }

        // 3. If the data passes our checks, fuse it into the SwerveDrive!
        if (isTrustworthy) {
            m_robotContainer.getDrivebase().addVisionMeasurement(
                llMeasurement.pose, 
                llMeasurement.timestampSeconds
            );
        }
    }
  }


  

  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

@Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // --- ALLIANCE SMART LIMELIGHT (AUTO) ---
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight", 0); 
        
        // Direct NetworkTable Override (Bypasses old LimelightHelpers)
        double[] redHubIDs = {2.0, 3.0, 4.0, 5.0, 8.0, 9.0, 10.0, 11.0}; 
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(redHubIDs);
    } else {
        LimelightHelpers.setPipelineIndex("limelight", 1); 
        
        double[] blueHubIDs = {18.0, 19.0, 20.0, 21.0, 24.0, 25.0, 26.0, 27.0};
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(blueHubIDs);
    }

    if (m_autonomousCommand != null) { m_autonomousCommand.schedule(); }
  }

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null) { m_autonomousCommand.cancel(); } 
    else { CommandScheduler.getInstance().cancelAll(); }

    // --- ALLIANCE SMART LIMELIGHT (TELEOP) ---
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        LimelightHelpers.setPipelineIndex("limelight", 0); 
        
        double[] redHubIDs = {2.0, 3.0, 4.0, 5.0, 8.0, 9.0, 10.0, 11.0}; 
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(redHubIDs);
    } else {
        LimelightHelpers.setPipelineIndex("limelight", 1); 
        
        double[] blueHubIDs = {18.0, 19.0, 20.0, 21.0, 24.0, 25.0, 26.0, 27.0};
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(blueHubIDs);
    }
  }
  

  @Override
  public void autonomousPeriodic() {}



  @Override
  public void teleopPeriodic()
  {
    // --- BUTTON CONTROL LOGIC ---
    // Check if the 'X' button is being held down on the main controller
    //if (driverController.getXButton()) {
      // If held, spin motor at 50% speed
    //  testMotor.set(0.5); 
   // } else {
      // If released, stop the motor
      //testMotor.set(0);
    }
    // ----------------------------
  

  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}