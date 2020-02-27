/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax intakeSweeperMotor;
  CANSparkMax intakeKickerMotor;
  DoubleSolenoid extender;


  public IntakeSubsystem() {
    intakeSweeperMotor = new CANSparkMax(RobotMap.INTAKE_SWEEPER_MOTOR, MotorType.kBrushless);
    intakeKickerMotor = new CANSparkMax(RobotMap.INTAKE_KICKER_MOTOR, MotorType.kBrushless);
    intakeKickerMotor.setIdleMode(IdleMode.kBrake);
    intakeSweeperMotor.setIdleMode(IdleMode.kBrake);


    extender = new DoubleSolenoid(6,7);

  }

   public void extendExtender() {
     extender.set(Value.kForward);
   }

  public void retractExtender() {
    extender.set(Value.kReverse);

  }

  public void setIntakeMotors(double input) {
    intakeSweeperMotor.set(input);
  }

  public void setKickerMotor(double input){
    intakeKickerMotor.set(-input);
  }
 
  public void raiseIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void lowerIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
