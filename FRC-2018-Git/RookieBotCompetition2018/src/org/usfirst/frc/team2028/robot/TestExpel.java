package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

public class TestExpel extends Command {
	Gripper gripper;
	double gripperspeed;
	private double seconds;
	double[] speeds = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
	
	public TestExpel(Gripper gripper_, int knobspeed_) {
		gripper = gripper_;
		gripperspeed = speeds[knobspeed_-1];
		seconds = Parameters.GRIPPER_SPITTIME;
	}
	
	protected void initialize()
	{
		this.setTimeout(seconds);
	}

	protected void execute()
	{
		gripper.ejectCube(gripperspeed);
	}

	@Override
	protected boolean isFinished()
	{
		return this.isTimedOut();
		//			in theory this will work, but this will have to be tested on the robot
	}
	protected void end()
	{

	}
}

