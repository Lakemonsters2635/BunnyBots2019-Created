/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.models.FMSInfo;
/**
 * Add your docs here.
 */
public class ColorSpinnerSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  CANSparkMax colorSpinnerMotor;
  State state = State.DISABLED;
  Color expectedColor = ColorMatcher.kRedTarget;
  double target_rotation = 0;
  
  private Color m_targetColor; 
  public String targetColorName;
  private DigitalInput contactSwitch;

  public ColorSpinnerSubsystem() {
    colorSpinnerMotor = new CANSparkMax(RobotMap.CONTROL_PANEL_MOTOR, MotorType.kBrushless);
    colorSpinnerMotor.setIdleMode(IdleMode.kBrake);
    contactSwitch = new DigitalInput(1);
  }

  ColorMatcher matcher = new ColorMatcher();

  

  enum State {
      DISABLED,
      ENC_ROTATE,
      COLOR_ROTATE,
      COLOR_ROTATE_FINAL,
  };

  

  public void update() {
    // FireLog.log("colorwheelpos", drive.getEncoder().getPosition());
  }

  public void stop() {
      state = State.DISABLED;
  }
  public boolean isContactDetected() {
    return contactSwitch.get();
  }

  public void find_color() {
      if ( state != State.COLOR_ROTATE ) {
          state = State.COLOR_ROTATE;
      }
  }

  Color[] ColorWheel = new Color[] {
      ColorMatcher.kGreenTarget,
      ColorMatcher.kRedTarget,
      ColorMatcher.kYellowTarget,
      ColorMatcher.kBlueTarget
  };

  public void Init() {
  }

  public void Periodic() {
    // seq.update();
      //SmartDashboard.putNumber("ColorSpinner Distance", GetMotorDistance());
  }

  public Color getColorSensorColor() {
      return matcher.get_color();
  }
  public Color getTargetColor() {
      return m_targetColor;
  }

  public void spinToTargetColor(){
      colorSpinnerMotor.set(0.2);
  }

  public void stopTargetSpinner() {
      colorSpinnerMotor.set(0.0);
  }

  public boolean spinFinished(){
      Color colorFound = matcher.get_color();
      //NOTE: Discuss the following with students
      //What is the difference between comparing objects with "==" or "colorFound.equals"
      boolean isFinished = (colorFound.equals(m_targetColor));

      if (isFinished){
          colorSpinnerMotor.set(0);
      }
      return isFinished;
  }

  public boolean determineTargetColor(){
    targetColorName = "Unknown";
    FMSInfo fmsInfo = getFMSInfo();
    if (fmsInfo.isInitalized) {
        switch(fmsInfo.controlPanelTargetColor) {
            case 'R':
                m_targetColor = (Color)ColorMatcher.m_colorDictionary.get(ColorMatcher.kRedTarget);
                targetColorName = "Red";
                break;
            case 'G':
                m_targetColor = (Color)ColorMatcher.m_colorDictionary.get(ColorMatcher.kGreenTarget);
                targetColorName = "Green";
                break;
            case 'B':
                m_targetColor = (Color)ColorMatcher.m_colorDictionary.get(ColorMatcher.kBlueTarget);
                targetColorName = "Blue";
                break;   
            case 'Y':
                m_targetColor = (Color)ColorMatcher.m_colorDictionary.get(ColorMatcher.kYellowTarget);
                targetColorName = "Yellow";
            break;                     
            default:
                return false;
        }
        return true;
    }  else {
        return false;
    }
}



public static FMSInfo getFMSInfo()
{
    FMSInfo fmsInfo = new FMSInfo();
    try {
        DriverStation driveStation= DriverStation.getInstance();
    
        System.out.println("FMS Attached: " + driveStation.isFMSAttached());
            
        //FHE: How do we test "IsFMSAttached())
        
        String gameSpecificMessage = "";

            
        //driveStation.waitForData(); //FHE: DO WE NEED THIS?
        fmsInfo.alliance = driveStation.getAlliance();
        gameSpecificMessage = driveStation.getGameSpecificMessage();

        fmsInfo.controlPanelTargetColor = gameSpecificMessage.trim().toUpperCase().charAt(0);
        fmsInfo.driveStation = driveStation.getLocation();
        fmsInfo.isAutonomous = driveStation.isAutonomous();
        fmsInfo.isInitalized = true;
    } 
    
    catch(Exception err) {
        System.out.println("ERROR Getting FMS Info: " + err.getMessage());
    }

return fmsInfo;
}
@Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
