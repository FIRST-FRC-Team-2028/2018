package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LineCamera {
	DigitalInput red;
	DigitalInput blue;
	DigitalInput white;
	DigitalInput black;
	
	LineCamera()
	{
		red = new DigitalInput(9);
		blue = new DigitalInput(8);
		white = new DigitalInput(7);
		black = new DigitalInput(6);
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
}
