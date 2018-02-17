package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

public class PlaceCubeCommand extends Command
{	
	Gripper gripper;
	double gripperspeed;
	private double seconds;
	public PlaceCubeCommand(Gripper gripper_, double gripperspeed_)
	{
		gripper = gripper_;
		gripperspeed = gripperspeed_;
		seconds = Parameters.GRIPPER_SPITTIME;
	}

	protected void initialize()
	{
		this.setTimeout(seconds);
	}

	protected void execute()
	{
		gripper.ejectCube();
	}
	
	@Override
	protected boolean isFinished()
	{
		return this.isTimedOut();
//		in theory this will work, but this will have to be tested on the robot
	}
	protected void end()
	{
		
	}
}

