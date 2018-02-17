package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoOption5 extends CommandGroup {

	/**
	 *From the side drop the cube on nearest available target
	 *at least cross auto line 
	 */
    public AutoOption5(PIDController pidcontroller, Drive drive,Lift lift , Gripper gripper, Ultrasonic ultrasonic, LineCamera Linecam, double wait_time, 
			int knobposition, boolean left, String gamedata)
    {	    	
    		boolean leftdriverswitch;
		leftdriverswitch = (knobposition > 7)? true: false;
    			
    		char robotSide = (leftdriverswitch)? 'R':'L';
    		  
//    		If the switch is ours put cube on it
    		if (gamedata.charAt(0) == robotSide)  
    		{
//    			run autooption 2
    			addSequential(new AutoOption2( pidcontroller,  drive,  ultrasonic,  wait_time, 
    					 knobposition, left, gamedata));
    		}  
//    		If the scale is ours put cube on it
    		else if (gamedata.charAt(1) == robotSide)  
    		{
//			run autooption4 
    			addSequential(new AutoOption4( pidcontroller,  drive, lift, gripper, ultrasonic, Linecam, wait_time,
   					 knobposition, left, gamedata));
    		}
//    		cross autoline
    		else
    		{
//    		run autooption1
    			addSequential(new AutoOption1( pidcontroller,  drive,  wait_time, 
      					 knobposition, left));
    		}
    }
    
    }
