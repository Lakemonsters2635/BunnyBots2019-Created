/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK_CHANNEL);
  public Joystick leftStick = new Joystick(RobotMap.LEFT_JOYSTICK_CHANNEL);

  public Button referenceResetButton = new JoystickButton(leftStick, 8);

  Button shooterVisionButton = new JoystickButton(leftStick, RobotMap.SHOOTER_WITHVISION_BUTTON);
  Button shooterNoVisionButton = new JoystickButton(rightStick, RobotMap.SHOOTER_NOVISION_BUTTON);

  Button intakeDetectButton = new JoystickButton(leftStick, 2);

  Button visionButton = new JoystickButton(leftStick, RobotMap.VISION_BUTTON_BUTTON);
  

  Button elevatorUpButton = new JoystickButton(rightStick, 5);
  Button elevatorDownButton = new JoystickButton(rightStick, 4);

  Button elevatorIndexUpButton = new JoystickButton(rightStick, 3);
  Button elevatorIndexDownButton = new JoystickButton(rightStick, 2);

  Button intakeInButton = new JoystickButton(rightStick, 10);
  Button intakeOutButton = new JoystickButton(rightStick, 11);

  Button lowerIntakeButton = new JoystickButton(leftStick, 11);
  Button raiseIntakeButton = new JoystickButton(leftStick, 10);

  Button lowerShooterButton = new JoystickButton(leftStick, 7);
  Button raiseShooterButton = new JoystickButton(leftStick, 6);


  Button helloArcButton = new JoystickButton(rightStick, 7);

  Button climberExtendButton  = new JoystickButton(rightStick, 8);
  Button climbButton = new JoystickButton(rightStick, 9);

  Button colorSpinnerButton = new JoystickButton(leftStick, 9);

}
