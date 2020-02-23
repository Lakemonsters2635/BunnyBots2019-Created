/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  DoubleSolenoid leftClimber;
  DoubleSolenoid rightClimber;
  CANSparkMax winchMotor;


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    leftClimber = new DoubleSolenoid(0, 1);
    rightClimber = new DoubleSolenoid(2, 3);
    winchMotor = new CANSparkMax(RobotMap.CLIMBER_WINCH_MOTOR,MotorType.kBrushless);
  }

  public void extendPistons() {
    leftClimber.set(Value.kForward);
    rightClimber.set(Value.kForward);
    
  }
  public void retractPistons() {
    leftClimber.set(Value.kReverse);
    rightClimber.set(Value.kReverse);
  }
  public void climb() {
    winchMotor.setVoltage(1);
  }

  public void stopClimbing() {
    winchMotor.setVoltage(0);
  }
}
