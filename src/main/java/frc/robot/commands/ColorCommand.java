/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class ColorCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorSpinnerSubsystem m_colorSpinner;


  /**
   * Creates a new ColorCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ColorCommand(ColorSpinnerSubsystem subsystem) {
    m_colorSpinner = subsystem;
 
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    boolean targetColorDetected = m_colorSpinner.determineTargetColor();
    if (targetColorDetected ){
      System.out.println("color command initalized");
    } else {
        System.out.println("Could not determine target color from FMS.");
        end();
    }
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //FHE: Do we need this?
    m_colorSpinner.spinToTargetColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean colorFound =  m_colorSpinner.spinFinished();
    if (colorFound) {
      System.out.println("Color '" +  m_colorSpinner.targetColorName + "' found.");
    }
    return colorFound;
  }


  
}
