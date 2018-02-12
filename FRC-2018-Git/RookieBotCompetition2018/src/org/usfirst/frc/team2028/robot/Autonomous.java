package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
//	LineCamera color;
//	
//	Autonomous()
//	{
//		color = new LineCamera();
//	}
//	
//	
//	public static void leftPositionAutoLine(Drive drive)
//	{
//		drive.driveToPosition(100);
//	}
//	public void middlePositionAutoLine(Drive drive)
//	{
//		drive.driveToPosition(100);
//	}
//	public void rightPositionAutoLine(Drive drive)
//	{
//		drive.driveToPosition(100);
//	}
//	/////////////////////////////////////////////////////////////////////////////////////////////////
//	/**
//	 * The Autonomous mode for if the robot is set up on the left side of the field (facing from the driver station)
//	 * and is going to place a cube on the switch.
//	 * 
//	 * @param switchleft false if our alliance's switch position is on the right, true if our alliance's switch
//	 * 			position in on the left
//	 */
//	public static void leftPositionSwitch(boolean switchleft, Drive drive, PIDController pidcontroller)
//	{
//		if(switchleft)
//		{
////			SmartDashboard.putNumber("right position drive",drive.getrightposition());
////			SmartDashboard.putNumber("Left Position Motor", drive.getleftposition());
////			SmartDashboard.putBoolean("pidcontroller enabled?", pidcontroller.isEnabled());
////			if(!drive.driveToPosition(100))
////			drive.driveToPosition(100);
////			pidcontroller.enable();
////			SmartDashboard.putBoolean("pidcontroller enabled?", pidcontroller.isEnabled());
////			pidcontroller.setSetpoint(Robot.turnTo(90));
////			drive.pidRotate();
////			pidcontroller.disable();
////			drive.driveToPosition(10);
////			drive.go();
//		}
//		else
//		{
//			drive.driveToPosition(10);
//			pidcontroller.enable();
//			pidcontroller.setSetpoint(Robot.turnTo(90));
//			drive.pidRotate();
//			pidcontroller.disable();
//			drive.driveToPosition(50);
//			pidcontroller.enable();
//			pidcontroller.setSetpoint(Robot.turnTo(0));
//			drive.pidRotate();
//			pidcontroller.disable();
//			drive.driveToPosition(90);
//		}
//	}
//	
//	public void leftPositionScale(boolean scaleleft, Drive drive, PIDController pidcontroller)
//	{
//		if(scaleleft)
//		{
//			
//		}
//	}
//	////////////////////////////////////////////////////////////////////////////////////////////
//	public void middlePositionSwitch(boolean switchleft)
//	{
//		
//	}
//	public void middlePositionScale(boolean scaleleft)
//	{
//		
//	}
//	//////////////////////////////////////////////////////////////////////////////////////////////
//	public void rightPositionSwitch(boolean switchleft)
//	{
//		
//	}
//	public void rightPositionScale(boolean scaleleft)
//	{
//		
//	}
}
