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
    m_upperMotorSpeed = RobotMap.SHOOTER_MOTOR_LOW_DEFAULT_SPEED;
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
      if (visionTargetFound) {

          targetDistance = Robot.vision.getXDistance();
          motor1Speed = computeShooterSpeedFromTargetDistance(targetDistance, shootHigh);
      } 
    }




    
    //THE FOLLOWING COMMENTED OUT CODE USES THE JOYSTICK SLIDERS TO ADJUST MOTOR SPEED
    // double motor1Adjust = RobotContainer.oi.leftStick.getRawAxis(3);
    // double motor2Adjust = RobotContainer.oi.rightStick.getRawAxis(3);
    // motor1Speed = motor1Speed + (1000 * motor1Adjust);
    // motor2Speed = motor1Speed + (1000 * motor2Adjust);

    //SmartDashboard.putNumber("ShooterMotor1", motor1Speed);
     //motor1Speed =  SmartDashboard.getNumber("ShooterMotor1", RobotMap.SHOOTER_MOTOR_LOW_DEFAULT_SPEED);
     Robot.shooterSubsystem.SpinShooter(motor1Speed);
  }

  public double computeShooterSpeedFromTargetDistance(double targetDistance, boolean isShooterHigh) {
    double adjustedMotorSpeed;
    if(isShooterHigh) {
      adjustedMotorSpeed = 2.0693 * targetDistance + 1418.6;
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
    return super.isTimedOut();
  }
}
