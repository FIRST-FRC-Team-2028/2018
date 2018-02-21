package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2028.robot.Parameters.TapeColor;

public class LineCamera {
	DigitalInput red;
	DigitalInput blue;
	DigitalInput white;
	DigitalInput black;
	
	LineCamera()
	{
		red = new DigitalInput(TapeColor.RED.getDigitalInputChannel());
		blue = new DigitalInput(TapeColor.BLUE.getDigitalInputChannel());
		white = new DigitalInput(TapeColor.WHITE.getDigitalInputChannel());
		black = new DigitalInput(TapeColor.BLACK.getDigitalInputChannel());
	}
	
	public boolean isRed()
	{
		return red.get();
	}
	
	public boolean isBlue()
	{
		return blue.get();
	}
	
	public boolean isWhite()
	{
		return white.get();
	}
	
	public boolean isBlack()
	{
		return black.get();
	}
	
	/**
	 * returns 1 if the camera sees red, 2 if it sees white, 3 if blue, and 4 if black.
	 * returns 0 if it does not see anything.
	 * @return
	 */
	public TapeColor getColor()
	{
		if(isRed())
		{
			return TapeColor.RED;
		}
		if(isWhite())
		{
			return TapeColor.WHITE;
		}
		if(isBlue())
		{
			return TapeColor.BLUE;
		}
		if(isBlack())
		{
			SmartDashboard.putString("Line Camera Vision", "black");
			return TapeColor.BLACK;
		}
		return TapeColor.NONE;
	}
}
