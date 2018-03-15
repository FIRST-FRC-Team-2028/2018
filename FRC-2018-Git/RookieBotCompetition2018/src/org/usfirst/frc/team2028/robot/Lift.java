package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Lift extends Subsystem{

	WPI_TalonSRX Lift_Motor;
	boolean fwdlimitclosed;
	double setpoint;

	Lift()
	{
		if(Parameters.LIFT_AVAILABLE)
		{

			Lift_Motor = new WPI_TalonSRX(Parameters.CanId.LIFTER_MASTER.getCanId());
			Lift_Motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
//			Lift_Motor.config_kP(0, Parameters.Pid.LIFT.getP(), 0);
//			Lift_Motor.config_kI(0, Parameters.Pid.LIFT.getI(), 0);
//			Lift_Motor.config_kD(0, Parameters.Pid.LIFT.getD(), 0);
//			Lift_Motor.config_kF(0, Parameters.Pid.LIFT.getF(), 0);
//			Lift_Motor.set(ControlMode.Position, 0);
			Lift_Motor.setNeutralMode(NeutralMode.Brake);
			Lift_Motor.setInverted(Parameters.CanId.LIFTER_MASTER.isInverted());
			Lift_Motor.setSensorPhase(true);
			//Can we limit the motor speed or ramp the speed?
			Lift_Motor.configClosedloopRamp(Parameters.LIFT_secondsFromNeutralToFull, 0);

			//			Lift_Motor.configForwardSoftLimitThreshold((int)Parameters.LIFT_TOP_POSITION, 0);
			//			Lift_Motor.configForwardSoftLimitEnable(true, 0);

			Lift_Motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
			Lift_Motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
			//		Lift_Motor.configReverseSoftLimitEnable(false, 0);
			//		Lift_Motor.configForwardSoftLimitEnable(false, 0);
		}
	}
	public double getVoltage()
	{
		if(Parameters.LIFT_AVAILABLE){
			return Lift_Motor.getMotorOutputVoltage();
		}
		return 0;
	}
	//
	//	public void engageRatchet()
	//	{
	//		if(Parameters.LIFT_AVAILABLE)
	//		{
	//			solenoid.set(Value.kForward);
	//		}
	//	}
	//	
	//	public void disengageRatchet()
	//	{
	//		if(Parameters.LIFT_AVAILABLE)
	//		{
	//			solenoid.set(Value.kReverse);
	//		}
	//	}

	//	public void climb()
	//	{
	//		if(Parameters.LIFT_AVAILABLE)
	//		{
	//			
	//		}
	//
	//	}

	public void setVoltage(double speed)
	{
		Lift_Motor.set(speed);
	}
	
	public double getSetpoint()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			return setpoint;
		}
		return 0;
	}

	public double getPosition()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			return Lift_Motor.getSensorCollection().getQuadraturePosition();
		}
		return 0;
	}

	public void setPosition(double switchPosition)
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			Lift_Motor.set(ControlMode.Position, switchPosition);
			setpoint = switchPosition;
		}
	}

	public void zeroPosition()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			Lift_Motor.getSensorCollection().setQuadraturePosition(0, 0);
		}
	}

	public boolean isLiftUp()
	{
		if(Parameters.LIFT_AVAILABLE)
		{	
			return Lift_Motor.getSensorCollection().isFwdLimitSwitchClosed();
		}
		return false;
	}

	public boolean isAtSetPoint()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			double range = Parameters.LIFT_TOP_POSITION - Parameters.LIFT_BOTTOM_POSITION;
			double current_percentage = (getPosition() - Parameters.LIFT_BOTTOM_POSITION) / range;
			double setpoint_percentage = (setpoint - Parameters.LIFT_BOTTOM_POSITION) / range;
			double diff = Math.abs(current_percentage - setpoint_percentage);
			if(diff <= Parameters.LIFT_POSITION_THRESHOLD)
			{
				return true;
			}
			return false;
		}
		return false;
	}

	public void stopMotor()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			Lift_Motor.stopMotor();
		}
	}

	public boolean isLiftDown()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			return Lift_Motor.getSensorCollection().isRevLimitSwitchClosed();
		}
		return false;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
