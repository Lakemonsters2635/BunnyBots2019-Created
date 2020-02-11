/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.ColorSensorV3;

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
  public ColorSpinnerSubsystem() {
    colorSpinnerMotor = new WPI_TalonSRX(RobotMap.COLOR_SPINNER_MOTOR);
    colorSpinnerMotor.setNeutralMode(NeutralMode.Brake);
  }

  ColorMatcher matcher = new ColorMatcher();

  private Color m_targetColor; 


  enum State {
      DISABLED,
      ENC_ROTATE,
      COLOR_ROTATE,
      COLOR_ROTATE_FINAL,
  };
  WPI_TalonSRX colorSpinnerMotor;
  State state = State.DISABLED;
  Color expectedColor = ColorMatcher.kRedTarget;
  double target_rotation = 0;

  public void update() {
    // FireLog.log("colorwheelpos", drive.getEncoder().getPosition());
  }

  public void stop() {
      state = State.DISABLED;
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

  public void spinToTargetColor(){
      colorSpinnerMotor.set(0.2);
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

  public void determineTargetColor(){
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    byte index;

      FMSInfo fmsInfo = getFMSInfo();
      if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
        case 'G':
            index = 0;
            break;
        case 'R':
            index = 1;
            break;
        case 'Y':
            index = 2;
            break;
        case 'B':
            index = 3;
            break;
        default:
            index = 100;
            break;
        }
    } else {
        index = 100;
    }

    //start the color finding
    if(index >= 4) {
        System.out.println("No color was provided to spin to.");
    } else {
        //need to rotate around the table.  We look at it from the far left, and need to rotate it 3 slots left, or 1 right, over to be what needs to be under the sensor.
        index = (byte)((index + 1) % 4);
    
        //seq.addStep(new FindColorWheelSlot(ColorWheel[index]));
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
