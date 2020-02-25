/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class ColorSpinCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorSpinnerSubsystem m_colorSpinner;


  /**
   * Creates a new ColorCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ColorSpinCommand(ColorSpinnerSubsystem subsystem) {
    m_colorSpinner = subsystem;
 
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
      System.out.println("color spin command initalized");
     
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
    m_colorSpinner.stopTargetSpinner();
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  protected void interrupted() {
    
    super.interrupted();
    end();
  }

  
}
