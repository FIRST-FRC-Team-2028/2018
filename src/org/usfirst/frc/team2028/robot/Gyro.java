package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro extends Subsystem implements PIDSource {
	private ADXRS450_Gyro gyro;
	
	Gyro()
	{
		if (Parameters.GYRO_AVAILABLE)
		{
			gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		if (Parameters.GYRO_AVAILABLE) 
		{
			gyro.setPIDSourceType(pidSource);
		}
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		if (Parameters.GYRO_AVAILABLE) {
			return gyro.getPIDSourceType();
		} else
		{
			return PIDSourceType.kRate;
		}
	}

	@Override
	public double pidGet(){
		
		// TODO Auto-generated method stub
		if (Parameters.GYRO_AVAILABLE) {
			SmartDashboard.putNumber("GYROANGLE", gyro.getAngle());
			return gyro.getAngle();
		} else {
			return 0.0;
		}
	}
	
	public double getAngle()
	{
		if (Parameters.GYRO_AVAILABLE) {
			return gyro.getAngle();
		} else
		{
			return 0.0;
		}			
	}
	
	public void reset()
	{
		if (Parameters.GYRO_AVAILABLE) {
			gyro.reset();
		}
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}