package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoOption2 extends CommandGroup 
{
	/**
	 * Deposits cube on switch from a side position (if the side 
	 * we are on matches our alliance color)
	 */
	public AutoOption2(PIDController pidcontroller, Drive drive, Ultrasonic ultrasonic, double wait_time, 
			int knobposition, boolean left, String gamedata)
	{
		boolean leftSwitch = (gamedata.charAt(0) == 'L')? true : false;
		boolean leftdriverswitch = (knobposition > 7)? false : true;
		
//	 	drive forward across auto line(auto option 1)
		addSequential(new AutoOption1(pidcontroller, drive, wait_time, knobposition, leftdriverswitch));

//		Check if switch is our alliance color
//		drive forward until robot approx 1/2 width of plate
//			D1 = DISTANCE auto line to driver station = 120 in
//			D2 = DISTANCE fence to driver station = 140 in
//			D3 = DISTANCE width of fence = 56 in
//			Drive Distance  = (D2 - D1 + 0.5 * D3)
//							= (20 + 0.5 * 56)
//							= 48 in
		if(!leftdriverswitch && leftSwitch || leftdriverswitch && !leftSwitch)
		{
			return;
		}else
		{
			
		addSequential(new DriveCommand(drive, 48));
		
		
//		Turn to face switch
//		based off of start position
//		if start position is left turn right
//		if start position is right turn left
//		= +/- 90 degrees
		double turn90 = (left)? 90 : -90;
			addSequential(new RotateCommand(drive, pidcontroller, turn90));
		
//		dive till bumper touches fence
//		(the ultrasonic provides distance)
		addSequential(new DriveUltrasonicCommand(drive, ultrasonic, Parameters.CUBEHANDLER_LENGTH));
//		deposits cube on switch
		
//		addSequential(new PlaceCubeCommand())//To be made
		}
	}
	
}
