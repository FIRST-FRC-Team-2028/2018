package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2028.robot.Parameters.TapeColor;

public class DriveAcrossLineCommand extends Command {

	Drive drive;
	TapeColor linecolor;
	LineCamera linecam;
	
	public DriveAcrossLineCommand(Drive drive_, TapeColor linecolor_, LineCamera linecam_)
	{
		drive = drive_;
		linecolor =linecolor_;
		linecam =linecam_;
	}
	
	protected void initialize()
	{
		
	}
	
	protected void execute()
	{
		drive.go();
		drive.set(Parameters.LINE_SEARCH_SPEED);
	}
	
	@Override
	protected boolean isFinished() 
	{
//		TODO  getLine method in PixyCamera
		switch (linecolor)
		{
		case BLACK: 
			return linecam.isBlack();
			
		case RED:
			return linecam.isRed();
		
		case BLUE: 
			return linecam.isBlue();
		
		case WHITE:
			return linecam.isWhite();
		}
		return false;
	}
	
	protected void end()
	{
		drive.stop();
		drive.go();
	}

}
