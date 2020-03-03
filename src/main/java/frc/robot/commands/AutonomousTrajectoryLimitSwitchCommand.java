package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.models.AutonomousTrajectories;

import java.util.ArrayList;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.Trajectory.Segment;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.Side;



/**
 *
 */
public class AutonomousTrajectoryLimitSwitchCommand extends AutonomousTrajectoryCommand {

    public AutonomousTrajectoryLimitSwitchCommand(Trajectory trajectory) {
        super(trajectory);
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
        super.initialize();
    
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        super.execute();

    }

    // Called once after timeout
    protected void end() {
        super.end();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
    @Override protected boolean isFinished() {       


        
        boolean isFinished = super.isFinished();
        if (!isFinished()) {
            isFinished = Robot.colorSpinnerSubsystem.isContactDetected();
        }

    	return isFinished;
    }
}