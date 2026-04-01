// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot
{
  private static Robot instance;
  private Command m_autonomousCommand;
  private int visionLoopCounter = 0;

  private RobotContainer m_robotContainer;

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
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }


@Override
public void robotPeriodic()
{
  CommandScheduler.getInstance().run();
  double omegaRps = m_robotContainer.getDrivebase()
      .getSwerveDrive()
      .getFieldVelocity()
      .omegaRadiansPerSecond;

  // Get Limelight pose estimate
  var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

  // Validate measurement + reject if spinning too fast
  if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {

    double averageTagDistance = llMeasurement.avgTagDist;

    boolean isTrustworthy = false;

    // Multi-tag = more reliable
    if (llMeasurement.tagCount >= 2) {
      if (averageTagDistance < 4.5) {
        isTrustworthy = true;
      }
    }
    // Single-tag = only trust when close
    else if (llMeasurement.tagCount == 1) {
      if (averageTagDistance < 2.5) {
        isTrustworthy = true;
      }
    }

    // Fuse vision into pose estimator
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
  }
  
  @Override
  public void disabledPeriodic()
  {
    m_robotContainer.setMotorBrake(false);
  }

  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // --- NEW: Trigger the homing routine safely! ---
    m_robotContainer.shooter.startHoming();
    
    LimelightHelpers.setPipelineIndex("limelight", 0); 
        double[] LimelightIDs = {1.0, 2.0, 4.0, 5.0, 6.0, 7.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 20.0, 21.0, 22.0, 23.0, 26.0, 28.0, 29.0, 30.0, 31.0, 32.0}; 
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getTable("limelight").getEntry("fiducial_id_filters_set").setDoubleArray(LimelightIDs);

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
