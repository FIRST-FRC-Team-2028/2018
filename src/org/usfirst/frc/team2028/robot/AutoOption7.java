package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * from center starting position
 * deposit cube on switch side,
 * get another cube from power cube zone
 */
public class AutoOption7 extends CommandGroup {

    public AutoOption7(PIDController cameraController, PIDController pidcontroller,Gripper gripper, Drive drive, Lift lift, Ultrasonic ultrasonic, double wait_time, int knobposition,
    		boolean left, String gamedata) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
    	
    	//Auto Option 3 to place a cube on the switch, with the robot facing the opposite alliance.
    	addSequential(new AutoOption3(pidcontroller, drive, lift, gripper, ultrasonic, wait_time, knobposition, left, gamedata));
    	
    	// backup a little and lower gripper
    	addSequential(new ResetGripperPosition(drive, lift));
    	
    	// turn toward the power cube zone to make detection of cube easier
    	
    	//uses drive follow command to look for a cube and stops when the cube is detected in the gripper.
    	addSequential(new DriveToCubeCommand(drive, ultrasonic, pidcontroller, gripper));
    	
    }
}