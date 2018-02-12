package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Lift extends Subsystem{
	
	WPI_TalonSRX Lift_Motor;
	
	boolean fwdlimitclosed;
	double setpoint;
	
	Lift()
	{
		Lift_Motor = new WPI_TalonSRX(30);
		Lift_Motor.config_kP(1, 1, 0);
		Lift_Motor.config_kI(1, 0.00002, 0);
		Lift_Motor.config_kD(1, 0, 0);
		Lift_Motor.config_kF(1, 0, 0);
		Lift_Motor.set(ControlMode.Position, 0);
		Lift_Motor.setNeutralMode(NeutralMode.Brake);
		
		
		
		Lift_Motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		Lift_Motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
//		Lift_Motor.configReverseSoftLimitEnable(false, 0);
//		Lift_Motor.configForwardSoftLimitEnable(false, 0);
	}
	
	public double getSetpoint()
	{
		return setpoint;
	}
	public double getPosition()
	{
		return Lift_Motor.getSensorCollection().getQuadraturePosition();
	}
	
	public void setPosition(double switchPosition)
	{
		Lift_Motor.set(ControlMode.Position, switchPosition);
		setpoint = switchPosition;
	}
	
	public void zeroPosition()
	{
		Lift_Motor.getSensorCollection().setQuadraturePosition(0, 0);
	}
	
	public boolean isLiftUp()
	{
		return Lift_Motor.getSensorCollection().isFwdLimitSwitchClosed();
	}
	
	public boolean isAtSetPoint()
	{
//		double distance = setpoint - Lift_Motor.getSensorCollection().getQuadraturePosition();
//		double delta = distance/ Lift_Motor.get - ();
//		return (== Parameters.LIFT_POSITION_THRESHOLD)? true :false;
		return false;
	}
	
//	public double setpoint()
//	{
//		
//	}
	public void stopMotor()
	{
		Lift_Motor.stopMotor();
	}
	
	public boolean isLiftDown()
	{
		return Lift_Motor.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	public void resetPosition()
	{
		Lift_Motor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	 
}
