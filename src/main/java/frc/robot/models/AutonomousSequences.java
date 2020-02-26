package frc.robot.models;

import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutonomousTrajectoryCommand;
import frc.robot.commands.IntakeCommand;

public class AutonomousSequences {



	public static CommandGroup ShootThenCollectRight(){
		CommandGroup output = new CommandGroup();
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