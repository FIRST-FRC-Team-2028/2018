package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic {
	
	AnalogInput ultrasonic;
	Ultrasonic(int port)
	{
		ultrasonic = new AnalogInput(port);
	}
	
	public double getDistance()
	{
		return ((ultrasonic.getValue()-59.55)/7.727)+6;
	}
	
	public double speed()
	{
		int distance = (int)(getDistance()/10);
		switch(distance)
		{
		case 0: return 0;
		case 1: return 200;
		case 2: return 300;
		case 3: return 400;
		case 4: return 450;
		case 5: return 500;
		case 6: return 550;
		case 7: return 650;
		default: return 700;
		}
	}
	
}
