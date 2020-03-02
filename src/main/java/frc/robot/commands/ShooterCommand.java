/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private boolean useCamera = false;
  private double targetDistance = 0;
  private double m_upperMotorSpeed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(boolean useCamera) {
    this.useCamera = useCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  //Constructor for Autonomous, contains a timeout and motorspeed.
  public ShooterCommand(boolean useCamera, double timeout, double upperMotorSpeed) {
    super(timeout);
    this.useCamera = useCamera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
   // m_colorSpinner.determineTargetColor();
      System.out.println("shooter command initalized. Use-Vision:" + useCamera);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double motor1Speed = RobotMap.SHOOTER_MOTOR_1_DEFAULT_SPEED;

    if (useCamera) {

      Robot.vision.data();
      boolean visionTargetFound = Robot.vision.targetExists();
      if (visionTargetFound) {
          targetDistance = Robot.vision.getXDistance();
          //System.out.println("target distance: " + targetDistance);
          motor1Speed = computeShooterSpeedFromTargetDistance(targetDistance);
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

     motor1Speed =  SmartDashboard.getNumber("ShooterMotor1", RobotMap.SHOOTER_MOTOR_1_DEFAULT_SPEED);
     Robot.shooterSubsystem.SpinShooter(motor1Speed);
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
    Robot.shooterSubsystem.stop();
    
    if (useCamera) {
      Robot.vision.ledOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isTimedOut();
  }
}
