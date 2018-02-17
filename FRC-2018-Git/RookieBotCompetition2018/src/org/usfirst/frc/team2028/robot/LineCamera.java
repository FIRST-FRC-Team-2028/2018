package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DigitalInput;
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
}
