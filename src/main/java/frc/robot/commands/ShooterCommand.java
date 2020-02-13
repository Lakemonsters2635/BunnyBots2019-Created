/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_Shooter;
  private double targetDistance = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem subsystem) {
    m_Shooter = subsystem;
 
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
   // m_colorSpinner.determineTargetColor();
      System.out.println("shooter command initalized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double motor1Speed = RobotMap.SHOOTER_MOTOR_1_DEFAULT_SPEED;
    double motor2Speed = RobotMap.SHOOTER_MOTOR_2_DEFAULT_SPEED;

    if (Robot.vision.isLightOn()) {

      Robot.vision.data();
      boolean visionTargetFound = Robot.vision.targetExists();
      if (visionTargetFound) {
          targetDistance = Robot.vision.getXDistance();
          //System.out.println("target distance: " + targetDistance);
          motor1Speed = computeShooterSpeedFromTargetDistance(targetDistance);
          motor2Speed = motor1Speed/2; //FOR TOP SPIN
          //System.out.println("Adjusted motor1 speed: " + motor1Speed);
      } else {
          //System.out.println("target not found");
      }
    }




    
    //THE FOLLOWING COMMENTED OUT CODE USES THE JOYSTICK SLIDERS TO ADJUST MOTOR SPEED
    // double motor1Adjust = RobotContainer.oi.leftStick.getRawAxis(3);
    // double motor2Adjust = RobotContainer.oi.rightStick.getRawAxis(3);
    // motor1Speed = motor1Speed + (1000 * motor1Adjust);
    // motor2Speed = motor1Speed + (1000 * motor2Adjust);

    //double motor1Speed =  SmartDashboard.getNumber("ShooterMotor1", 0);
    //double motor2Speed =  SmartDashboard.getNumber("ShooterMotor2", 0);

    //System.out.println("Motor1 Speed: " + motor1Speed);
    //System.out.println("Motor2 Speed: " + motor2Speed);
    m_Shooter.SpinShooter(motor1Speed, motor2Speed);
  }

  public double computeShooterSpeedFromTargetDistance(double targetDistance) {
    //FHE: TODO See google sheet with data.
    //https://docs.google.com/spreadsheets/d/1hFAy2s6HSixSz0b9KfWV3SxeLQ4VlyDaA7yp4tblp4I/edit#gid=0
    // Distance	motor 1 	motor 2
    // 301.75	  2,350	    1,225
    // 280	    2,350	    1,225
    // 250	    2,250	    1,125
    // 277	    2,400	    1200
    // 146 9/16	2100    	1050
    // 186	    2250    	1125
    // 211.25	  2300    	1150
    double adjustedMotorSpeed = 1.441875 * targetDistance + 1943.063021;
    return adjustedMotorSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
  m_Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_CommandInitializationFailed) {
    //   System.out.println("Command initalization failed. Finished immediately.");
    //   return true;
    // }

    //return m_colorSpinner.spinFinished();
    // boolean colorFound =  m_colorSpinner.isFinished(m_targetColor);
    // if (colorFound) {
    //   System.out.println("Color '" +  m_fmsInfo.controlPanelTargetColor + "' found.");
    // }
    // return colorFound;
    return false;
  }
}
