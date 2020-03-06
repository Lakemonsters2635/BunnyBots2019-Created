/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

  DoubleSolenoid climberSolenoid;
  TalonFX winchMotor;


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    climberSolenoid = new DoubleSolenoid(2, 3);
    winchMotor = new TalonFX(RobotMap.CLIMBER_WINCH_MOTOR);
    configureMotor();
    
  }

  public void extendPistons() {
    climberSolenoid.set(Value.kForward);
  }
  public void retractPistons() {
    climberSolenoid.set(Value.kReverse);
  }
  public void climb() {
    winchMotor.set(TalonFXControlMode.Velocity, 1500*2048/600);
  }

  public void stopClimbing() {
    winchMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  public void configureMotor(){
    winchMotor.setSensorPhase(true);

    winchMotor.configNominalOutputForward(0, RobotMap.kTimeoutMs);
    winchMotor.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
    winchMotor.configPeakOutputForward(0.8, RobotMap.kTimeoutMs);
    winchMotor.configPeakOutputReverse(-0.8, RobotMap.kTimeoutMs);


    winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                            RobotMap.kPIDLoopIdx, 
                                            RobotMap.kTimeoutMs);


    winchMotor.config_kF(RobotMap.kPIDLoopIdx, 0.0, RobotMap.kTimeoutMs);
    winchMotor.config_kP(RobotMap.kPIDLoopIdx, 0.1, RobotMap.kTimeoutMs);
    winchMotor.config_kI(RobotMap.kPIDLoopIdx, 0.0, RobotMap.kTimeoutMs);
    winchMotor.config_kD(RobotMap.kPIDLoopIdx, 0.0, RobotMap.kTimeoutMs);

}
}
