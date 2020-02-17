/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.commands.ZeroFieldOrientedCommand;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


//changes
//changes

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

 //To deploy this, run (gradlew deploy -x test) in the command prompt
public class Robot extends TimedRobot {
  //public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI oi;
  private static final double UPDATE_DT = 5e-3; // 5 ms

  public static DrivetrainSubsystem drivetrainSubsystem;

  public static Vision vision;

  public static ElevatorSubsystem elevatorSubsystem;
  public static IntakeSubsystem intakeSubsystem;

  public static ClimberSubsystem climberSubsystem;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private SubsystemManager subsystemManager;

  public HolonomicDriveCommand driveCommand;
  ZeroFieldOrientedCommand zeroCommand;


  ToggleDriveRecordCommand recordCommand;

  ElevatorUpCommand elevatorUpCommand;
  ElevatorDownCommand elevatorDownCommand;

  IntakeInCommand  intakeInCommand;
  IntakeOutCommand intakeOutCommand;


  VisionLightCommand visionLightCommand;
  VisionRotationDriveCommand visionRotationDriveCommand;
  LoadingBayToShootingCommand loadingBayToShootingCommand;
  public HelloArcCommand helloArcCommand;
  public static boolean arcCommandIsRunning = false;

  ExtendClimberCommand extendClimberCommand;
  ClimbCommand climbCommand;

  RobotRotateCommand robotRotateCommand;
  // IntakeCommandGroup intakeCommandGroup;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    //m_chooser.setDefaultOption("Default Auto", new AutonomousCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    vision = new Vision();
    drivetrainSubsystem = new DrivetrainSubsystem();
    // elevatorSubsystem = new ElevatorSubsystem();

    subsystemManager = new SubsystemManager(drivetrainSubsystem);
    // intakeSubsystem = new IntakeSubsystem();

    // climberSubsystem = new ClimberSubsystem();

    // recordCommand = new ToggleDriveRecordCommand();
    zeroCommand = new ZeroFieldOrientedCommand(drivetrainSubsystem);
    driveCommand = new HolonomicDriveCommand(DrivetrainSubsystem.ControlMode.DualStick);
    visionLightCommand = new VisionLightCommand();
    visionRotationDriveCommand = new VisionRotationDriveCommand();
    loadingBayToShootingCommand = new LoadingBayToShootingCommand(99);

    robotRotateCommand = new RobotRotateCommand(90);
    // elevatorUpCommand = new ElevatorUpCommand();
    // elevatorDownCommand = new ElevatorDownCommand();

    // intakeInCommand = new IntakeInCommand();
    // intakeOutCommand = new IntakeOutCommand();

    helloArcCommand = new HelloArcCommand();

    // intakeCommandGroup = new IntakeCommandGroup();

    // climbCommand = new ClimbCommand();
    // extendClimberCommand = new ExtendClimberCommand();
    
    //oi.bedForwardButton.toggleWhenPressed(bedForwardCommand);
    // oi.toggleDriveRecordButton.toggleWhenPressed(recordCommand);
    oi.visionButton.whileHeld(visionRotationDriveCommand);
    
    //oi.intakeButton.whileHeld(intakeCommandGroup);
    // oi.elevatorUpButton.whileHeld(elevatorUpCommand);
    // oi.elevatorDownButton.whileHeld(elevatorDownCommand);

    // oi.intakeInButton.whileHeld(intakeInCommand);
    // oi.intakeOutButton.whileHeld(intakeOutCommand);

    // oi.climberExtendButton.whenPressed(extendClimberCommand);
    // oi.climbButton.whileHeld(climbCommand);

    oi.helloArcButton.whileHeld(robotRotateCommand);

    //oi.bedReverseButton.toggleWhenPressed(bedReverseCommand);
    oi.referenceResetButton.whenPressed(zeroCommand);

    vision.ledOff();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    arcCommandIsRunning = helloArcCommand.isRunning();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Robot.drivetrainSubsystem.getFollower().cancel();

    Robot.drivetrainSubsystem.holonomicDrive(Vector2.ZERO, 0.0, true);

    subsystemManager.disableKinematicLoop();
    vision.ledOff();
    

    
  }

  

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    if(vision.getLed() != 1)
    vision.ledOff();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    
    subsystemManager.enableKinematicLoop(UPDATE_DT);
    zeroCommand.start();
    //m_autonomousCommand = m_chooser.getSelected();

    m_autonomousCommand = new AutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    //System.out.println(drivetrainSubsystem.getGyroscope().getRate());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    Robot.drivetrainSubsystem.getFollower().cancel();

    subsystemManager.enableKinematicLoop(UPDATE_DT);
    zeroCommand.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();
    
    Vector2 vec = drivetrainSubsystem.getKinematicPosition();
    //SmartDashboard.putNumber("Current Pose X", vec.x);
    //SmartDashboard.putNumber("Current Pose Y", vec.y);
   

    drivetrainSubsystem.outputToSmartDashboard();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    drivetrainSubsystem.outputToSmartDashboard();
  }
}
