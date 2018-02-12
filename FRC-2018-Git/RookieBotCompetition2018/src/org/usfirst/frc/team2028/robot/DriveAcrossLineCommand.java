package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveAcrossLineCommand extends Command {

	Drive drive;
	PixyCamera pixy;
	int linecolor;
	public DriveAcrossLineCommand(Drive drive_, PixyCamera pixy_, int linecolor_)
	{
		drive = drive_;
		pixy =pixy_;
		linecolor =linecolor_;
	}
	
	protected void initialize()
	{
		
	}
	
	protected void execute()
	{
		drive.go();
	}
	
	@Override
	protected boolean isFinished() 
	{
//		return (pixy.getLine(linecolor))? true : false;
//		TODO  getLine method in PixyCamera
		return false;
	}
	
	protected void end()
	{
		
	}

}
