package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoOption4 extends CommandGroup {

	/**
	 * From side position, cross the auto line,
	 *  drive into the null territory without crossing the middle line
	 *  and deposit the cube on the scale
	 */
    public AutoOption4(PIDController pidcontroller, Drive drive,Lift lift, Gripper gripper, Ultrasonic ultrasonic, LineCamera linecam, double wait_time, 
			int knobposition, boolean left, String gamedata)
    {
    	boolean leftdriverswitch;
    		leftdriverswitch = (knobposition > 7)? true: false;
    	
//		Drive to the Auto Line (run auto option 1)
		addSequential(new AutoOption1(pidcontroller, drive, wait_time, knobposition, leftdriverswitch));
		
//		goto scale and puts cube
		addSequential(new AutolineToScale( pidcontroller, drive, lift, gripper, ultrasonic, linecam, knobposition, left, gamedata));
    }
}
