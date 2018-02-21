package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PixyCamera implements PIDSource{
	AnalogInput X = new AnalogInput(Parameters.PIXY_CAMERA_CUBE_PORT);
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		X.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return X.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		return -((X.getAverageVoltage()-1.65))*20.0; //this is multiplied to manipulate the PIDController's output
	}												// to make the PIDController output larger values.
	public double getVoltage()
	{
		return X.getAverageVoltage();
	}
	
	public boolean iscubeseen()
	{
		if(getVoltage() < 0.03)
		{
			return false;
		}
		return true;
	}
	}

