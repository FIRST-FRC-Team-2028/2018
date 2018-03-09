package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoOption1 extends CommandGroup 
{
	/**
	 * Auto Option 1 takes the arbitrary starting position of the robot
	 * and then moves the  robot across the Auto Line.
	 * Once the robot has crossed the auto line, it will turn perpendicular.
	 */
	public AutoOption1 (PIDController pidcontroller, Drive drive, double wait_time, int knobposition, boolean left)
	{
		double pointDistance;
		if(left)
		{
			pointDistance = Parameters.FIELD_P11;
		}
		else
		{
			pointDistance = Parameters.FIELD_P1;
		}

		double D1 = 120;
		double angle;
		double pos;

		switch(knobposition)
		{
		case 1: 
			pos = 3.5;
			break;
		case 2: 
			pos = 5.5;
			break;
		case 3: 
			pos = 7.5;
			break;
		case 4: 
			pos = 9.5;
			break;
		case 5: 
			pos = 11.5;
			break;
		case 6: 
			pos = 13.5;
			break;
		case 7: 
			pos = 15.5;
			break;
		case 8: 
			pos = 17.5;
			break;
		case 9: 
			pos = 19.5;
			break;
		case 10: 
			pos = 21.5;
			break;
		case 11: 
			pos = 23.5;
			break;
		default: 
			pos = 3.5;
		}

		angle = Math.atan((pointDistance - pos)/(D1/12)); 
		
		
		addSequential(new WaitCommand(wait_time));

		//	Drive to set distance (across the auto line)
		//		 if the robot is in the center.
		//			D1 = DISTANCE driver station wall to auto line = 120 in
		//			A1 = ANGLE robot initial facing (relative to arcade center line)
		//				A1 = 0 (perpendicular to driver station wall)
		//				A1 = +/-30~ (facing center of switch plate from W8)
		//				A1 = +/- 52~ ( facing the edge of the auto line from W8)
		//		distance= D1/sin(A1)
		//		Parameter: distance till robot is past the auto line
		addSequential(new DriveCommand(drive, D1/Math.sin(angle)));

//		addSequential(new StopRobot(drive));
		//		Turn perpendicular to the drive station
		//		Parameter: turning angle
		addSequential(new TimedRotateCommand(200, -angle, drive));
		addSequential(new RotateCommand(drive, pidcontroller, -angle));
	}
}