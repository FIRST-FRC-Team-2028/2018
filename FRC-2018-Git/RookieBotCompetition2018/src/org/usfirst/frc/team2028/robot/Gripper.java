package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalOutput;

public class Gripper {
	int LEFT_GRIPPER_CAN_ID = 98;
	int RIGHT_GRIPPER_CAN_ID = 99;
	int GRIPPER_ANGLE_CAN_ID = 93;
	WPI_TalonSRX Left_Motor;
	WPI_TalonSRX Right_Motor;
	WPI_TalonSRX Angle_Motor;
	boolean cubeswitch = Left_Motor.getSensorCollection().isFwdLimitSwitchClosed();
	Gripper()
	{
		Left_Motor = new WPI_TalonSRX(LEFT_GRIPPER_CAN_ID);
		Left_Motor.set(ControlMode.PercentOutput, 0);
		Left_Motor.setNeutralMode(NeutralMode.Brake);
		
		Right_Motor = new WPI_TalonSRX(RIGHT_GRIPPER_CAN_ID);
		Right_Motor.setNeutralMode(NeutralMode.Brake);
		Right_Motor.follow(Left_Motor);
		Right_Motor.setInverted(true);
		
		
		Angle_Motor = new WPI_TalonSRX(GRIPPER_ANGLE_CAN_ID);
		Angle_Motor.set(ControlMode.Position, 0);
		Angle_Motor.setNeutralMode(NeutralMode.Brake);
	}
	
	public boolean isCubeHeld()
	{
		return Left_Motor.getSensorCollection().isFwdLimitSwitchClosed();
	}
	
	public void pickupCube(double speed)
	{
		Left_Motor.set(speed);
	}
	
	public void placeCube(double speed)
	{
		Left_Motor.set(-speed);
	}

	public void tilt(double pos)
	{
		Angle_Motor.set(ControlMode.Position, pos);
	}
}