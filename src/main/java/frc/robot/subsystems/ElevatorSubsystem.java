/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax beltMotor;
  CANSparkMax bottomKickerMotor;
  CANSparkMax topKickerMotor;
  
  DigitalInput bottomSensor;

  public ElevatorSubsystem() {
    beltMotor = new CANSparkMax(RobotMap.ELEVATOR_MOTOR_CHANNEL, MotorType.kBrushless);
    bottomKickerMotor = new CANSparkMax(0, MotorType.kBrushless);
    topKickerMotor = new CANSparkMax(RobotMap.UPPER_KICKER_MOTOR, MotorType.kBrushless);

    bottomSensor = new DigitalInput(0);
  }

  public void setBeltMotor(double input) {
    beltMotor.set(input);
  }
  
  public void setBottomKickerMotor(double input) {
    bottomKickerMotor.set(input);
  }

  public void setTopKickerMotor(double input) {
    topKickerMotor.set(input);
  }

  public boolean isBlocked() {
    return bottomSensor.get();
  }
  
  public void shooterLoad() {
    beltMotor.set(1);
    topKickerMotor.set(1);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
