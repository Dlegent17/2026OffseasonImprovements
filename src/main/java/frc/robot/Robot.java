// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot
{
  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  @Override
  public void robotInit()
  {
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();
    System.out.println("XXXX");
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
    
    // --- NEW: Trigger the homing routine safely! ---
    m_robotContainer.shooter.startHoming();
    
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

    // --- NEW: Trigger the homing routine safely! ---
    // (It will instantly skip if it already homed during Auto)
    m_robotContainer.shooter.startHoming();

        LimelightHelpers.setPipelineIndex("limelight", 0); 
        double[] LimelightIDs = {1.0, 2.0, 4.0, 5.0, 6.0, 7.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 20.0, 21.0, 22.0, 23.0, 26.0, 28.0, 29.0, 30.0, 31.0, 32.0}; 
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(LimelightIDs);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

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
