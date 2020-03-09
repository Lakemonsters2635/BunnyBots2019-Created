/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class RotateControlPanelCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorSpinnerSubsystem m_colorSpinner;
  private final int m_numberOfRotations;
  private int m_colorFoundCount = 0;
  private Color m_startColor;
  private Color m_currentColor;


  /**
   * Creates a new ColorCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateControlPanelCommand(ColorSpinnerSubsystem subsystem, int numberOfRotations) {
    m_colorSpinner = subsystem;
    m_numberOfRotations = numberOfRotations;
 
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_colorFoundCount = 0;
    m_startColor = m_colorSpinner.getColorSensorColor();
    m_currentColor = m_startColor;
    // if (m_startColor == Color.kBlack ){
    //   System.out.println("Could not determine current color from Color Sensor.");
    //   end();      

    // } else {
    //   System.out.println("RotateControlPanelCommand initalized");
    // }
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_colorSpinner.spinToTargetColor();
    boolean colorHasChanged = false;
    Color color = m_colorSpinner.getColorSensorColor();
    if (color != m_currentColor) {
      m_currentColor = color;
      colorHasChanged = true;
    }

    if (m_currentColor == m_startColor && colorHasChanged) {
      m_colorFoundCount++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    System.out.println("stop CPM Motor");
    m_colorSpinner.stopTargetSpinner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_colorFoundCount > m_numberOfRotations*2) ;
  }

  @Override
  public void interrupted() {
    end();
  }

  
}
