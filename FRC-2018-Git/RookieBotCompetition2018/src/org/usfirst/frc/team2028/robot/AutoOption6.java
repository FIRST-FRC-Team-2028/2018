package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoOption6 extends CommandGroup 
{
	/**
	 * from center deposit cube on scale
	 */
	public AutoOption6(PIDController pidcontroller, Drive drive, Lift lift, Gripper gripper, Ultrasonic ultrasonic, LineCamera Linecam, double wait_time, 
			int knobposition, boolean left, String gamedata)
	{
		boolean leftSwitch = (gamedata.charAt(0) == 'L')? true : false;
		
//		wait for robots to get out of the way
		addSequential(new WaitCommand(wait_time));
		
//		drives 10 inches forward 
		addSequential(new DriveCommand(drive, 10));
		
//		turns to set angle based off of which side of the scale is ours
//			D1= DISTANCE driver station wall auto line = 120 in
//			D2= DISTANCE arcade centerline to edge of fence = 74.25 in
//			D3 =DISTANCE from arcade center to edge of arcade = 162 in
//			D4 = DISTANCE driven = 10 inches
//			D5 = DISTANCE length of robot = 39.25 in
//				D4.1 length of chassis = 32.75 inches
//				D4.2 length of bumper = 6.5 in
//			ANGLE (driving to the middle between the fence and the edge of the arcade)
//			ANGLE = arctan((D2+(D2-D3)/2)/D1-D4-D5)
//			ANGLE = 23.3
		double angle = (leftSwitch)? -23.3 : 23.3;
		addSequential(new TimedRotateCommand(200, angle, drive));
		addSequential(new RotateCommand(drive, pidcontroller, angle));
		
//		Drives across the auto line 
//		DISTANCE =sqrt( (D2+(D2-D3)/2)^2 + (D1-D4-D5)^2)
//		DISTANCE =sqrt( 30.375^2 + 70.75^2)
//		DISTANCE ~= 77 in
		addSequential(new DriveCommand(drive, 77));
		
//		turn perpendicular to the driver station
		addSequential(new TimedRotateCommand(200, -angle, drive));
		addSequential(new RotateCommand(drive, pidcontroller, -angle));
		
//		goto scale and puts cube
		addSequential(new AutolineToScale( pidcontroller, drive, lift, gripper, ultrasonic, Linecam, knobposition, left, gamedata));

	}
	
	
}