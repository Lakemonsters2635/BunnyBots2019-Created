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
import frc.robot.commands.AutonomousTrajectoryLimitSwitchCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateControlPanelCommand;
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
                Path driveThroughTrenchPath = new Path(Rotation2.ZERO);
                driveThroughTrenchPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(130.271-21.007, 0) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveThroughTrenchTrajectory = new Trajectory(driveThroughTrenchPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveThroughTrenchCommand = new AutonomousTrajectoryCommand(driveThroughTrenchTrajectory);

                //drive 130.271-21.007

                IntakeCommand intakeCommand = new IntakeCommand(false, 2);
                output.addParallel(driveThroughTrenchCommand);
                output.addParallel(intakeCommand);
                output.addParallel(elevatorCommand);
                return output;
        }
        public static CommandGroup PositionForCPMAndRotateFourTimes(){
                CommandGroup output = new CommandGroup();
                Path driveOffWallPath = new Path(Rotation2.ZERO);
                driveOffWallPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(0, 27.5) //FHE:TODO Confirm positive/negative
                        )
                );


                Trajectory driveOffWallTrajectory = new Trajectory(driveOffWallPath, Robot.drivetrainSubsystem.CONSTRAINTS);

                AutonomousTrajectoryCommand driveOffWallCommand = new AutonomousTrajectoryCommand(driveOffWallTrajectory);

                output.addSequential(driveOffWallCommand);

                Path driveTowardsCPPath = new Path(Rotation2.ZERO);
                driveTowardsCPPath.addSegment(
                        new PathLineSegment(
                                new Vector2(0.0, 0.0),
                                new Vector2(200, 0) //FHE:TODO Confirm positive/negative
                        )
                );

                
                Trajectory driveTowardsCPTrajectory = new Trajectory(driveTowardsCPPath, Robot.drivetrainSubsystem.CONSTRAINTS);//TODO add slow constrains

                AutonomousTrajectoryLimitSwitchCommand driveTowardsCPCommand = new AutonomousTrajectoryLimitSwitchCommand(driveTowardsCPTrajectory);
                output.addSequential(driveTowardsCPCommand);

                RotateControlPanelCommand rotateControlPanelCommand = new RotateControlPanelCommand(Robot.colorSpinnerSubsystem, 4);
                output.addSequential(rotateControlPanelCommand);
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