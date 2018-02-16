package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Gripper {
	WPI_TalonSRX Left_Motor;
	WPI_TalonSRX Right_Motor;
	WPI_TalonSRX Angle_Motor;
	//	boolean cubeswitch = Left_Motor.getSensorCollection().isFwdLimitSwitchClosed();
	Gripper()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor = new WPI_TalonSRX(Parameters.CanId.LEFT_GRIPPER.getCanId());
			Left_Motor.set(ControlMode.PercentOutput, 0);
			Left_Motor.setNeutralMode(NeutralMode.Brake);
			Left_Motor.setInverted(Parameters.CanId.LEFT_GRIPPER.isInverted());


			Right_Motor = new WPI_TalonSRX(Parameters.CanId.RIGHT_GRIPPER.getCanId());
			Right_Motor.setNeutralMode(NeutralMode.Brake);
			Right_Motor.follow(Left_Motor);
			Right_Motor.setInverted(Parameters.CanId.RIGHT_GRIPPER.isInverted());


			Angle_Motor = new WPI_TalonSRX(Parameters.CanId.LIFT_TILT.getCanId());
			Angle_Motor.set(ControlMode.Position, 0);
			Angle_Motor.setNeutralMode(NeutralMode.Brake);
			Angle_Motor.setInverted(Parameters.CanId.LIFT_TILT.isInverted());

			Angle_Motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
					LimitSwitchNormal.NormallyClosed, 0);
			Angle_Motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
					LimitSwitchNormal.NormallyOpen, 0);
		}
	}

	public boolean isCubeHeld()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			return Left_Motor.getSensorCollection().isFwdLimitSwitchClosed();
		}
		return true;
	}

	public boolean isGripperUp()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			return Angle_Motor.getSensorCollection().isFwdLimitSwitchClosed();
		}
		return true;
	}

	public boolean isGripperDown()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			return Angle_Motor.getSensorCollection().isRevLimitSwitchClosed();
		}
		return true;
	}

	public void pickupCube()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor.set(Parameters.GRIPPER_INFEED_SPEED);
		}
	}

	public void launchCube()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor.set(Parameters.GRIPPER_LAUNCH_SPEED);
		}
	}

	public void ejectCube()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor.set(Parameters.GRIPPER_EJECT_SPEED);
		}
	}

	public void dribbleCube()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor.set(Parameters.GRIPPER_DRIBBLE_SPEED);
		}
	}

	public void setVerticalUp()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Angle_Motor.set(ControlMode.PercentOutput, Parameters.GRIPPER_TILT_SPEED);
		}
	}

	public void setVerticalDown()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Angle_Motor.set(ControlMode.PercentOutput, -Parameters.GRIPPER_TILT_SPEED);
		}
	}

	public void tiltUp()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Angle_Motor.set(ControlMode.PercentOutput, Parameters.GRIPPER_TILT_SPEED);
		}
	}

	public void tiltDown()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Angle_Motor.set(ControlMode.PercentOutput, -Parameters.GRIPPER_TILT_SPEED);
		}
	}

	public void stopGripper()
	{
		if(Parameters.GRIPPER_AVAILABLE){
			Left_Motor.set(0);
		}
	}
	//	public void tiltHorizontal()
	//	{
	//		Angle_Motor.set(ControlMode.Position, Parameters.GRIPPER_TILT_HORIZONTAL_POSITION);
	//	}
}