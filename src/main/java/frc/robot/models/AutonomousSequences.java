package frc.robot.models;

import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;

public class AutonomousSequences {



	public static CommandGroup ShootThenCollectRight(){
                CommandGroup output = new CommandGroup();
                ElevatorCommand elevatorCommand = new ElevatorCommand(false, 3);
                ShooterCommand shooterCommand = new ShooterCommand(false, 4, RobotMap.SHOOTER_INTITIATION_LINE_UPPER_MOTOR_SPEED );
                Path driveToTrenchPath = new Path(Rotation2.ZERO);
                driveToTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(84.63, -66.905) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveToTrenchTrajectory = new Trajectory(driveToTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveToTrenchCommand = new AutonomousTrajectoryCommand(driveToTrenchTrajectory);



                output.addParallel(elevatorCommand);
                output.addParallel(shooterCommand);
                output.addSequential(driveToTrenchCommand);
                
                //We've reached the trench. Now collect power cell. 
                IntakeCommand intakeCommand = new IntakeCommand(false, 2);
                output.addParallel(intakeCommand);
                output.addParallel(elevatorCommand);
                return output;
    }

    //Lifts intake
    //Drives forward 5 inches
    //Spins intake
    //Lowers intake and calls elevator state machine
    //Drives backward 5 inches.
    public static CommandGroup CollectPowerCell(){
        Path fiveInchesPath = new Path(Rotation2.ZERO);
        fiveInchesPath.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(5, 0)
                )
        );


        Trajectory fiveInchesForward = new Trajectory(fiveInchesPath, Robot.drivetrainSubsystem.CONSTRAINTS);


        CommandGroup output = new CommandGroup();
        
        AutonomousTrajectoryCommand trajectoryCommand = new AutonomousTrajectoryCommand(fiveInchesForward);
        IntakeCommand intakeCommand = new IntakeCommand(false);
        output.addParallel(intakeCommand);
        output.addParallel(trajectoryCommand);
	return output;
    }


    public static String getMethodName()
	{
		String methodName = Thread.currentThread().getStackTrace()[2].getMethodName();
		return methodName;
	}
}