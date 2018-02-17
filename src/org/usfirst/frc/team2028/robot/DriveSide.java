package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSide {
	
	/** this is a motor*/
	private WPI_TalonSRX Master;
	
	/** This is a TalonSRX that controls a Follower Motor that follows Master*/
	private WPI_TalonSRX Follower;
	
	DriveSide(int CAN_ID_Master, int CAN_ID_Follower, boolean isInverted, boolean setPhase)
	{
		
		Master = new WPI_TalonSRX(CAN_ID_Master);
		Master.setNeutralMode(NeutralMode.Brake);
		Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		Master.setSensorPhase(setPhase);
		Master.config_kP(0, Parameters.Pid.DRIVE_MOTOR.getP(), 0);
		Master.config_kI(0, Parameters.Pid.DRIVE_MOTOR.getI(), 0);
		Master.config_kD(0, Parameters.Pid.DRIVE_MOTOR.getD(), 0);
		Master.config_kF(0, Parameters.Pid.DRIVE_MOTOR.getF(), 0);
		Master.set(ControlMode.Velocity, 0);
		Master.setInverted(isInverted);

		Follower = new WPI_TalonSRX(CAN_ID_Follower);
		Follower.set(ControlMode.Follower, 0);
		Follower.setNeutralMode(NeutralMode.Brake);
		Follower.follow(Master);
		Follower.setInverted(isInverted);
	}

	//double position = Master.getSelectedSensorPosition(0);
	public double getPosition()
	{
		//SmartDashboard.putNumber("sensor position", Master.getSelectedSensorPosition(0));
		return Master.getSelectedSensorPosition(0)/165.0;
	}
	public double getVoltage(boolean master)
	{
		if (master) {
			return Master.getMotorOutputVoltage();
		} else {
			return Follower.getMotorOutputVoltage();
		}
	}
    
	public void resetPosition()
	{
		Master.getSensorCollection().setQuadraturePosition(0, 0);
	}
	
	public void setPosition(double pos)
	{
		Master.set(ControlMode.Position, pos);
	}
	
	public void set(double speed)
	{
		Master.set(ControlMode.Velocity, speed);
	}

	public void setVoltage(double speed)
	{
		Master.set(ControlMode.PercentOutput, speed);
	}	
}