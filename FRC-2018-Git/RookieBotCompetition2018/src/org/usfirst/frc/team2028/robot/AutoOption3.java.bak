package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoOption3 extends CommandGroup {

	/**
	 * Deposits cube on switch from a center position
	 */
	public AutoOption3(PIDController pidcontroller, Drive drive, Ultrasonic ultrasonic, double wait_time, 
			int knobposition, boolean left, String gamedata  ) 
	{
		boolean leftSwitch = (gamedata.charAt(0) == 'L')? true : false;

//		wait for robots to get out of the way
		addSequential(new WaitCommand(wait_time));

		
//		drives 10 inches forward 
			addSequential(new DriveCommand(drive, 10));
			
//		turns to set angle based off of which side of the switch is ours
//			D1= DISTANCE driver station wall to fence = 140 in
//			D2= DISTANCE arcade centerline to plate center = 54 in
//			D3 = DISTANCE driven = 10 inches
//			PointB = next point (near the scale) to which to drive
//			Bx = 84 +/- 54
//			By = D1 - Stand-off-distance 
//			ANGLE = arctan((By - D3)/(Bx-Robot location))

			double By = 140 - 
					(Parameters.DISTANCE_TO_SPIT + 
					 Parameters.DISTANCE_TO_SWITCH_FENCE + 
					 Parameters.ROBOT_LENGTH/2.);
			double Bx = (leftSwitch)? 30 : 138;
			double auto1angle; 
			auto1angle= Math.atan((By-10)/(Bx-84));   //robot starts in center ie x = 84 

			addSequential(new RotateCommand(drive, pidcontroller, auto1angle ));
			
//		drives till ultrasonic reads a set distance from the switch fence
			double standOffD = (Parameters.DISTANCE_TO_SPIT + 
					 Parameters.DISTANCE_TO_SWITCH_FENCE);
			double ultraSwitchDis = (standOffD/(Math.cos(auto1angle)));
			addSequential(new DriveUltrasonicCommand(drive, ultrasonic, ultraSwitchDis));
			
//		turn perpendicular to the drive station
			addSequential(new RotateCommand(drive, pidcontroller, -auto1angle));

			
//		drives forward till bumper touches the fence
			addSequential(new DriveUltrasonicCommand(drive, ultrasonic, Parameters.CUBEHANDLER_LENGTH));

			
//		put cube on switch plate
//			addSequential(new PlaceCubeCommand()); 

	}

	public AutoOption3(String name) {
		super(name);
		// TODO Auto-generated constructor stub
	}

}
	