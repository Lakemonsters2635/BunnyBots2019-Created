/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleDriveRecordCommand extends Command {

  boolean hasExecuted = false;
  public ToggleDriveRecordCommand() {
    
     
  }

  @Override
  protected void initialize() {
    hasExecuted = false;
  }

  @Override
  protected void execute() {
    if(Robot.drivetrainSubsystem.isRecordingDrive) {
        //System.out.println("saving recording");
        try{
            Robot.drivetrainSubsystem.writeDriveLog();
        }
        catch(Exception err){
            //System.out.println("Error writing log files:" + err.getMessage());

        }
        Robot.drivetrainSubsystem.isRecordingDrive = false;
        hasExecuted = true;
      } else {
        //System.out.println("starting recording");
        Robot.drivetrainSubsystem.translationLog.clear();
        Robot.drivetrainSubsystem.rotationLog.clear();
        Robot.drivetrainSubsystem.isRecordingDrive = true;
        hasExecuted = true;
      }

  }
  

  @Override
  protected boolean isFinished() {
      return hasExecuted;
  }
}
