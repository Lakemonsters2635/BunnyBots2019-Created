/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
public class ShooterSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonFX motor1;
  TalonFX motor2;
  CANSparkMax topKickerMotor;
  DoubleSolenoid shootorSolenoid;
  // DoubleSolenoid rightSolenoid;
  public ShooterSubsystem(){
    motor1 = new TalonFX(RobotMap.SHOOTER_TOP_CAN);
    motor2 = new TalonFX(RobotMap.SHOOTER_BOTTOM_CAN);

    topKickerMotor = new CANSparkMax(RobotMap.UPPER_KICKER_MOTOR, MotorType.kBrushless);
    //topKickerMotor = new CANSparkMax(10, MotorType.kBrushless);

    shootorSolenoid = new DoubleSolenoid(4,5);
    //shootorSolenoid = new DoubleSolenoid(5,4);
    //shootorSolenoid = new DoubleSolenoid(3,4);
    //shootorSolenoid = new DoubleSolenoid(2,3);
    //shootorSolenoid = new DoubleSolenoid(3,2);
    //shootorSolenoid = new DoubleSolenoid(1,2);
    //shootorSolenoid = new DoubleSolenoid(0,1);

    configureMotors();  
  }

  public void aimHigh(){
    shootorSolenoid.set(Value.kForward);
    DoubleSolenoid.Value val = shootorSolenoid.get();
    System.out.println("shootorSolenoid.Value:" + val);
  }

  public void aimLow(){
    shootorSolenoid.set(Value.kReverse);
    DoubleSolenoid.Value val = shootorSolenoid.get();
    System.out.println("shootorSolenoid.Value:" + val);
  }
  



  public void configureMotors(){
      motor1.setSensorPhase(true);
      motor2.setSensorPhase(true);

      motor1.configNominalOutputForward(0, RobotMap.kTimeoutMs);
      motor1.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
      motor1.configPeakOutputForward(1, RobotMap.kTimeoutMs);
      motor1.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

      motor2.configNominalOutputForward(0, RobotMap.kTimeoutMs);
      motor2.configNominalOutputReverse(0, RobotMap.kTimeoutMs);
      motor2.configPeakOutputForward(1, RobotMap.kTimeoutMs);
      motor2.configPeakOutputReverse(-1, RobotMap.kTimeoutMs);

      motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                              RobotMap.kPIDLoopIdx, 
                                              RobotMap.kTimeoutMs);

      motor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                              RobotMap.kPIDLoopIdx, 
                                              RobotMap.kTimeoutMs);

      motor1.config_kF(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kF, RobotMap.kTimeoutMs);
      motor1.config_kP(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kP, RobotMap.kTimeoutMs);
      motor1.config_kI(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kI, RobotMap.kTimeoutMs);
      motor1.config_kD(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kD, RobotMap.kTimeoutMs);

      motor2.config_kF(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kF, RobotMap.kTimeoutMs);
      motor2.config_kP(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kP, RobotMap.kTimeoutMs);
      motor2.config_kI(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kI, RobotMap.kTimeoutMs);
      motor2.config_kD(RobotMap.kPIDLoopIdx, RobotMap.kGains_Velocit.kD, RobotMap.kTimeoutMs);


      topKickerMotor.setIdleMode(IdleMode.kBrake);
  }

  public void update() {
    // FireLog.log("colorwheelpos", drive.getEncoder().getPosition());
  }

  public void stop() {
    //  state = State.DISABLED;
    motor1.set(ControlMode.Velocity, 0);
    motor2.set(ControlMode.Velocity, 0);
    topKickerMotor.set(0);
  
  }

  public void Init() {
  }

  public void SpinShooter(double motor1Speed, double motor2Speed) {
      //DON'T GO OVER 3,000;
      motor1Speed = Math.min(5000, Math.abs(motor1Speed));
      motor2Speed = Math.min(5000, Math.abs(motor2Speed));

      motor1.set(ControlMode.Velocity, motor1Speed*2048/600);
      motor2.set(ControlMode.Velocity, -motor2Speed*2048/600);

      topKickerMotor.set(-1);
  }

  public void setTopKickerMotor(double input) {
    topKickerMotor.set(input);
  }

  public void shooterLoad() {
    topKickerMotor.set(1);
  }

  public double GetMotorDistance() {
      //return drive.getEncoder().getPosition();
      return 0;
  }
  // public Color getColor(){
  //    // return matcher.get_color();
      
  // }
  public void StartRotation() {
      //if a color wheel operation is going, don't change it
    //  if(seq.isRunning() == true) return;
  //    seq.addStep(new SpinColorWheel());
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
