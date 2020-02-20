/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class RobotRotateCommand extends Command {
  PIDController angleController;
  double angle;
  double currentAngle;

  //NOTE: This command rotates to an absolute angle based on the orientation the robot started in. Will work in Auto, must be adapted for Teleop
  public RobotRotateCommand(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(2);
    angleController = new PIDController(0.005, 0.001, 0.0);
    angleController.enableContinuousInput(-180, 180);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    angleController.setSetpoint(angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentAngle = Robot.drivetrainSubsystem.getGyroscope().getAngle().toDegrees();
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, angleController.calculate(currentAngle));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(currentAngle > angle - 2 && currentAngle < angle + 2 ) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
