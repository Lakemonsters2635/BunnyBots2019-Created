/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LimitSwitchCommand extends Command {


  public LimitSwitchCommand(double timeout) {
    super(timeout);
   
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //System.out.println("Elevator Down command init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean contactDetected =Robot.colorSpinnerSubsystem.isContactDetected();
    if (contactDetected) {
      System.out.println("Limit Switch Contact Deteted!");
    }
    
    return contactDetected;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("Elevator Down command end");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
