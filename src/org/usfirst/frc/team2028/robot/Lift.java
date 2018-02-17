package org.usfirst.frc.team2028.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Lift extends Subsystem{

	WPI_TalonSRX Lift_Motor;
	DoubleSolenoid solenoid;
	boolean fwdlimitclosed;
	double setpoint;

	Lift()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			solenoid = new DoubleSolenoid(Parameters.RATCHET_ENGAGE_CHANNEL,
					Parameters.RATCHET_DISENGAGE_CHANNEL);

			Lift_Motor = new WPI_TalonSRX(Parameters.CanId.LIFTER_MASTER.getCanId());
			Lift_Motor.config_kP(1, Parameters.Pid.LIFT.getP(), 0);
			Lift_Motor.config_kI(1, Parameters.Pid.LIFT.getI(), 0);
			Lift_Motor.config_kD(1, Parameters.Pid.LIFT.getD(), 0);
			Lift_Motor.config_kF(1, Parameters.Pid.LIFT.getF(), 0);
			Lift_Motor.set(ControlMode.Position, 0);
			Lift_Motor.setNeutralMode(NeutralMode.Brake);

			Lift_Motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
			Lift_Motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
			//		Lift_Motor.configReverseSoftLimitEnable(false, 0);
			//		Lift_Motor.configForwardSoftLimitEnable(false, 0);
		}
	}

	public void engageRatchet()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			solenoid.set(Value.kForward);
		}
	}
	
	public void disengageRatchet()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			solenoid.set(Value.kReverse);
		}
	}

	public void climb()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			
		}

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
			//		double distance = setpoint - Lift_Motor.getSensorCollection().getQuadraturePosition();
			//		double delta = distance/ Lift_Motor.get - ();
			//		return (== Parameters.LIFT_POSITION_THRESHOLD)? true :false;
		}
		return false;			//FIX ME: REPLACE WITH ACTUAL CALCULATION
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

	public void resetPosition()
	{
		if(Parameters.LIFT_AVAILABLE)
		{
			Lift_Motor.getSensorCollection().setQuadraturePosition(0, 0);
		}
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
