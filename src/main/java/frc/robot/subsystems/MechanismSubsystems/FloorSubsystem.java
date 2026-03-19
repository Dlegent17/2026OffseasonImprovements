package frc.robot.subsystems.MechanismSubsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorSubsystem extends SubsystemBase {

    // Motors
    
    private final SparkMax backBeltMotor;
    


    @SuppressWarnings("removal")
    // Constructor initializes motors and encoder, and configures motor settings like current limits and idle modes.
    public FloorSubsystem() {
        backBeltMotor = new SparkMax(16, MotorType.kBrushless);
        
        
        

        // 
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.smartCurrentLimit(30);
        rollerConfig.idleMode(IdleMode.kCoast);

        SparkMaxConfig backBeltConfig = new SparkMaxConfig();
        backBeltConfig.apply(rollerConfig);
        backBeltConfig.inverted(true);
        backBeltMotor.configure(backBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        // This runs constantly. It puts your exact intake angle on the dashboard 
        // so you can easily read it and find your real limits later!
        
    }
    /**
     * Reads the absolute encoder and converts it to degrees.
     * DutyCycleEncoders return 0.0 to 1.0 by default, so we multiply by 360.
     */
    
// A simple helper method to stop all intake motors.
    public void stopRollers() {
        //frontRollerMotor.set(0.0);
        backBeltMotor.set(0.0);
    }

    

    
    
    /**
     * A Command that simultaneously drops the intake and runs the rollers to pull in a game piece.
     * Drops the intake to the floor and spins the rollers.
     * The soft limits in setPivotSpeed() will automatically stop the arm when it hits the floor!
     */

     public void runIntake() {
        // frontRollerMotor.set(-0.7);
        backBeltMotor.set(-0.2);
     }

     public void stopIntake() {
        // frontRollerMotor.set(-0.7);
        backBeltMotor.set(0.0);
     }

     public Command fixedIntake() {
        return this.runEnd(this::runIntake, this::stopIntake);
     }
     
     

   

    

    

    /**
     * Pulls the intake back up into the robot and stops the rollers.
     * It finishes automatically when the absolute encoder says it has reached the top.
     */
    
}