/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.ControlMode;

public class VisionRotationDriveCommand extends Command {

  PIDController angleController;
  long lastTime;

  public VisionRotationDriveCommand() {
    requires(Robot.drivetrainSubsystem);
    //PidConstants PID_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    angleController = new PIDController(0.01, 0.01, 0);
  }

  @Override
  protected void initialize() {
    lastTime = System.currentTimeMillis();
  }

  @Override
  protected void execute() {
    Robot.vision.data();
    double forward = 0;
    double strafe = 0;
    double rotation = 0;

    double angle = Robot.vision.getXAngle();
    // boolean ignoreScalars =
    // Robot.oi.primaryController.getforwardBumperButton().get();
    
    forward = Robot.oi.leftStick.getRawAxis(1);
    strafe = Robot.oi.leftStick.getRawAxis(0);
    if (Robot.vision.targetExists()) {

      angleController.setSetpoint(angle);

      System.out.println("Found target  Angle: " + angle);
      // if (Math.abs(angle)>5) {
      //   if (angle>0){
      //     rotation = 0.2;
      //   } 
      //   else if (angle<0){
      //     rotation = -0.2;
      //   }
      // }

      rotation = angleController.calculate(0);
    }
    else {
      System.out.println("No target");
    }

    final double deadzone = 0.1;

    forward = deadZoneAdjust(forward, deadzone);
    strafe = deadZoneAdjust(strafe, deadzone);
    //rotation = deadZoneAdjust(rotation, deadzone);
    if(rotation > 1){
      rotation = 1;
    }else if(rotation < -1){
      rotation = -1;
    }

    final boolean robotOriented = false;
    final boolean reverseRobotOriented = false;

    final Vector2 translation = new Vector2(forward, strafe);

    // if (reverseRobotOriented) {
    // robotOriented = true;
    // translation = translation.rotateBy(Rotation2.fromDegrees(180.0));
    // }
    // System.out.println("HoloDriveCommand.execute" + translation + " " +
    // rotation);
    Robot.drivetrainSubsystem.holonomicDrive(translation, rotation, !robotOriented);
  }

  public double deadZoneAdjust(double input, final double deadzone) {
    if(input > deadzone) {
        input = ((input - deadzone)/(1.0 - deadzone));
      } else if(input < -deadzone) {
        input = ((input + deadzone)/(1.0 - deadzone));
      } else {
        input = 0;
      }    
    return input;
}

@Override
protected boolean isFinished() {
    return false;
}

@Override
  protected void end() {
    Robot.vision.ledOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
