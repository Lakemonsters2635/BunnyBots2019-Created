/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;

import org.frcteam2910.common.util.HolonomicDriveSignal;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

//-------------------------------------
//Swerve Drive Experiment Imports BEGIN
//-------------------------------------
//import frc.lib.motion_profiling.Autonomi;
//-------------------------------------
//Swerve Drive Experiment Imports END
//-------------------------------------
// import frc.lib.motion_profiling.Autonomous;
// import frc.lib.motion_profiling.Path2D;

public class AutonomousCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousCommand() {
    requires(Robot.drivetrainSubsystem);

    //-------------------------------------
    //Swerve Drive Experiment  BEGIN
    //-------------------------------------

    // try {

    //   String jsonString = new String(Files.readAllBytes(Paths.get("/home/lvuser/autonomi.json") ));
    //   Autonomi pathfollow = Autonomi.fromJsonString(jsonString);
    //   //System.out.println("Autonomi cache loaded.");
    //   if (pathfollow != null && pathfollow.mapAutonomous != null) {

    //     // using iterators 
    //     Iterator<Map.Entry<String, Autonomous>> itr = pathfollow.mapAutonomous.entrySet().iterator(); 
              
    //     while(itr.hasNext()) 
    //     { 
    //         Map.Entry<String, Autonomous> entry = itr.next(); 
    //         System.out.println("Key = " + entry.getKey() +  ", Value = " + entry.getValue().name); 
    //         Map<String, Path2D> paths = entry.getValue().paths;
    //         Iterator<Map.Entry<String, Path2D>> pathItr = paths.entrySet().iterator(); 
    //         while (pathItr.hasNext()) {
    //           Map.Entry<String, Path2D> currentPath = pathItr.next();
              
    //           System.out.println("pathName: " + currentPath.getValue().name);
    //         }

    //     } 
    //   }

      
    // } catch (Exception err) {
    //    System.out.println("Autonomi load failed: " + err.getMessage());
     
    // }




    // ArrayList<HolonomicDriveSignal> driveSequence = DrivetrainSubsystem.readDriveRecording("/home/lvuser/driveSequences/driveStraight.log", true); 
    // AutonomousDriveCommand autonomousDriveCmd = new AutonomousDriveCommand(driveSequence, 13);
    // addSequential(autonomousDriveCmd);

    AutonomousTrajectoryCommand trajectorCommand = new AutonomousTrajectoryCommand(4);
    addSequential(trajectorCommand);

    //DriveCommand driveCommand = new DriveCommand(translation, rotation, fieldOriented);


  }
}
