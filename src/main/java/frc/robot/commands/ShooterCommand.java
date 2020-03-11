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
  private boolean shootHigh = true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(boolean useCamera) {
    this.useCamera = useCamera;
    m_upperMotorSpeed = RobotMap.SHOOTER_MOTOR_HIGH_DEFAULT_SPEED;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  //Constructor for Autonomous, contains a timeout and motorspeed.
  public ShooterCommand(boolean useCamera, double timeout, double upperMotorSpeed) {
    super(timeout);
    this.useCamera = useCamera;
    m_upperMotorSpeed = upperMotorSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double motor1Speed = m_upperMotorSpeed;

    if (useCamera) {

      Robot.vision.data();
      boolean visionTargetFound = Robot.vision.targetExists();
      SmartDashboard.putBoolean("TargetFound", visionTargetFound);
      if (visionTargetFound) {

          targetDistance = Robot.vision.getXDistance();
          motor1Speed = computeShooterSpeedFromTargetDistance(targetDistance, shootHigh);
      } 
    } else {

        //For now, Only use throttle adjustments in high position.
        //If we want to use in low position we need to take measurements and calibrate.
        
          double motor1Adjust = Robot.oi.rightStick.getRawAxis(2);
          //Make no changes to motor speed unless the user has moved the throttle. 
       
              //Turn motor1Adjust into a range between 0 - 2.
              if (motor1Adjust >= 0) {
                motor1Adjust = 1 + motor1Adjust;
              } else {
                motor1Adjust = 1 - Math.abs(motor1Adjust);
              }
              SmartDashboard.putNumber("motor1Adjust", motor1Adjust);
              motor1Speed = 2500 - (315 * motor1Adjust);
              SmartDashboard.putNumber("motor1Speed", motor1Speed);
              //motor1Speed now ranges between 1700 - 1900, depending on the throttle. 
         



    }


     Robot.shooterSubsystem.SpinShooter(motor1Speed);
  }

  public double computeShooterSpeedFromTargetDistance(double targetDistance, boolean isShooterHigh) {
    double adjustedMotorSpeed;
    if(isShooterHigh) {
      adjustedMotorSpeed = 2.0693 * targetDistance + 1418.6;
      //adjustedMotorSpeed = 2.0693 * targetDistance + 1400.6;
    }
    else {
      adjustedMotorSpeed = 4000;
    }
    
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
    boolean isFinished = super.isTimedOut();
    if (isFinished) {
      System.out.println("ShooterCommand finished");
    }
    return isFinished;
    //return super.isTimedOut();
  }
}
