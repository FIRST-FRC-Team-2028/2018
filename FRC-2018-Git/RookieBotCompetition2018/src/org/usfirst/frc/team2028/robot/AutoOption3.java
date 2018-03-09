package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoOption3 extends CommandGroup {

	/**
	 * Deposits cube on switch from a center position
	 */
	public AutoOption3(PIDController pidcontroller, Drive drive, Lift lift, Gripper gripper, Ultrasonic ultrasonic, double wait_time, 
			int knobposition, boolean left, String gamedata  ) 
	{
		boolean leftSwitch = (gamedata.charAt(0) == 'L')? true : false;

//		wait for robots to get out of the way
		addSequential(new WaitCommand(wait_time));

		
//		drives 10 inches forward (to point c)
			addSequential(new DriveCommand(drive, 10));
			
//			Drive to point B to drop a cube
//		turns to set angle based off of which side of the switch is ours
//			D1= DISTANCE driver station wall to fence = 140 in
//			D2= DISTANCE arcade centerline to plate center = 54 in
//			  =  centerline +/- (switchlength/2 - platelength/2)
//			C = position of robot after initial drive 
//			Cy = 10 inches + 1/2 Robot Length
//			Cx = 1/2 field width
//			PointB = next point (near the scale) to which to drive
//			Bx = 1/2 field width +/- D2
//			By = D1 - Stand-off-distance 
//			ANGLE = arctan((By-Cy)/(Bx-Cx))

			double D2 = 54;
			double Cy = 10 + Parameters.ROBOT_LENGTH/2;
			double Cx = Parameters.FIELD_WIDTH/2;
			double By = 140 - (Parameters.GRIPPER_LOCATION + Parameters.CUBE_WIDTH/2);
			double Bx = (leftSwitch)? Cx - D2: Cx + D2;
			double auto1angle; 
			auto1angle= Math.atan((By-Cy)/(Bx-Cx));   //robot starts in center ie x = 84 
//			TODO check this atan function especially the minus Cx portion ( if Bx = 30 this portion will end up negative) 
			addSequential(new TimedRotateCommand(200, auto1angle, drive));
			addSequential(new RotateCommand(drive, pidcontroller, auto1angle ));
			
//		drives till ultrasonic reads a set distance from the switch fence
			double standOffD = Parameters.GRIPPER_LOCATION + 
					 Parameters.CUBE_WIDTH/2; 
			double ultraSwitchDis = (standOffD/(Math.cos(auto1angle))); // check the math here
			addSequential(new DriveUltrasonicCommand(drive, ultrasonic, ultraSwitchDis));
			
//			raise lift to switch position 
			addParallel(new LiftCommand(lift, Parameters.LIFT_SWITCH_POSITION));
			
//		turn perpendicular to the drive station
			addSequential(new RotateCommand(drive, pidcontroller, -auto1angle));

//		drives forward till bumper touches the fence
			addSequential(new DriveUltrasonicCommand(drive, ultrasonic, Parameters.CUBEHANDLER_LENGTH));

//		put cube on switch plate
			addSequential(new PlaceCubeCommand(gripper, Parameters.GRIPPER_EJECT_SPEED)); 

	}

	public AutoOption3(String name) {
		super(name);
		// TODO Auto-generated constructor stub
	}

}
	