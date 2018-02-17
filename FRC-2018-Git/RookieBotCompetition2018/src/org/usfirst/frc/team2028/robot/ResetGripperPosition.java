package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ResetGripperPosition extends CommandGroup {
	/**
	 * After having deposit cube 
	 * Back up and send gripper to bottom position
	 */

	public ResetGripperPosition(Drive drive, Lift lift) {
		// Back up some distance 10
		addSequential(new DriveCommand(drive, -10));
		
		//Send gripper to bottom position
		addSequential(new LiftCommand(lift, Parameters.LIFT_ZERO_POSITION));
	}
}
