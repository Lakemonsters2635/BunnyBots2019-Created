package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.models.AutonomousTrajectories;

import java.util.ArrayList;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.Side;



/**
 *
 */
public class LoadingBayToShootingCommand extends Command {

    Trajectory autonomousTrajectory;
  
    public LoadingBayToShootingCommand( double timeout) {
        super(timeout);
        requires(Robot.drivetrainSubsystem);
       
        AutonomousTrajectories trajectoryLibrary = new AutonomousTrajectories(Robot.drivetrainSubsystem.CONSTRAINTS);
        //autonomousTrajectory = trajectoryLibrary.getCargoSideMidToLoadingStationTrajectory(Side.LEFT);
        autonomousTrajectory = trajectoryLibrary.getLoadingBayToShootingTrajectory();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }
	
    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drivetrainSubsystem.getGyroscope().setAdjustmentAngle(Robot.drivetrainSubsystem.getGyroscope().getUnadjustedAngle());
        Vector2 position = new Vector2(0, 0);
        Robot.drivetrainSubsystem.resetKinematics(position, 0);


        Robot.drivetrainSubsystem.getFollower().follow(autonomousTrajectory);
 
        

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        //Robot.drivetrainSubsystem.setTargetVelocity(velocity);
           

    }

    // Called once after timeout
    protected void end() {
        Robot.drivetrainSubsystem.getFollower().cancel();
        System.out.println("end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
    @Override protected boolean isFinished() {       


        
        boolean isFinished = false;

        if(isTimedOut()){
            System.out.println("Timed Out");
            isFinished = true;
        }

    	return isFinished;
    }
}