package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2028.robot.Parameters.CanId;
import org.usfirst.frc.team2028.robot.Parameters;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.XboxController;

public class Drive extends Subsystem implements PIDOutput {
//	private Gyro gyro;
	/** Left side drive sub-sub-system, composition relationship */
	private DriveSide left;
	
	/** Composition relationship to right side drive sub-sub-system */
	private DriveSide right;
	
	/** Pneumantic solenoid to select low or high gear.  Composition relationship */
	private DoubleSolenoid shifter;
	
	private DoubleSolenoid ptoshift;
	/** 
	 * Default constructor.
	 */
	Drive()
	{
		shifter = new DoubleSolenoid(Parameters.LOW_GEAR, Parameters.HIGH_GEAR);
		ptoshift = new DoubleSolenoid(2, 3); //FIX ME! PUT IN PARAMETERS!!!!!!
		left = new DriveSide(CanId.LEFT_MASTER.getCanId(), 
				CanId.LEFT_FOLLOWER.getCanId(), 
				Parameters.LEFT_DRIVE_INVERTED, Parameters.LEFT_PHASE);
		right = new DriveSide(CanId.RIGHT_MASTER.getCanId(),
				CanId.RIGHT_FOLLOWER.getCanId(), 
				!Parameters.LEFT_DRIVE_INVERTED, Parameters.RIGHT_PHASE);
        shifter.set(DoubleSolenoid.Value.kForward);
	}
	double leftspeed;
	double rightspeed;
	boolean isvoltagemode;
	boolean ispositionmode;
	boolean isauto;
	double position;
	double value_;
	/**
	 * Method invoked by a PIDController class as part of its main
	 * calculation loop.  Only used when the robot is spinning in 
	 * place using the Gyro to turn to a specific angle.
	 * 
	 * @param value - Speed of the turn in ticks / time unit
	 */
	@Override
	public void pidWrite(double value)
	{
		SmartDashboard.putNumber("value...", value);
		
//		right.set(-value);
//		left.set(value);
		value_ = value;
	}

	public void driveFollow(double speed)
	{
//		right.set(-speed);
//		left.set(-(speed-value_));
		
		rightspeed = -speed;
		leftspeed = -(speed-value_);
		isvoltagemode = false;
	}
	
	public void go()
	{
		if(isvoltagemode)
		{
			right.setVoltage(rightspeed);
			left.setVoltage(leftspeed);
		}else {
			right.set(rightspeed);
			left.set(leftspeed);
		}
	}
	
	public void go(double leftgo, double rightgo)
	{
		right.set(rightgo);
		left.set(leftgo);
		SmartDashboard.putNumber("AutoPositionRight", right.getPosition());
		SmartDashboard.putNumber("AutoPositionLeft", left.getPosition());
	}
	public void stop()
	{
		rightspeed = 0;
		leftspeed = 0;
	}
	public void pidRotate()
	{
//		right.set(-value_);
//		left.set(value_);
		rightspeed = -value_;
		leftspeed = value_;
		isvoltagemode = false;
	}
	
	public void rotate(double speed)
	{
//		right.set(speed);
//		left.set(-speed);
		rightspeed = speed;
		leftspeed = -speed;
		isvoltagemode = false;
	}
	
	/**
	 * Overloaded method to drive straight ahead.  Invoked as part of autonomous.
	 * Uses the TalonSRX speed control mode to drive at the provided 
	 * speed.
	 * 
	 * @param value - Spee to drive straight forward in ticks / time unit
	 */
	public void set(double value)
	{
//		right.set(value);
//		left.set(value);
		rightspeed = value;
		leftspeed = value;
		isvoltagemode = false;
	}
	
	public double getVoltage(boolean isLeft, boolean master) {
		if (isLeft) {
			return left.getVoltage(master);
		} else {
			return right.getVoltage(master);
		}
	}

	/** Overloaded method to drive the robot using two inputs (e.g.,
	 * tank drive) Using one value for the left side and one
	 * value for the right side's speed
	 * 
	 * @param leftSpeed - Speed of the left side in ticks / time unit
	 * @param rightSpeed - Speed of the right side in ticks / time unit
	 */
	public void setVoltage(double leftSpeed, double rightSpeed)
	{
//		right.setVoltage(rightSpeed);
//		left.setVoltage(leftSpeed);
		rightspeed = rightSpeed;
		leftspeed = leftSpeed;
		isvoltagemode = true;
	}

	public void speedControl(double speed)
	{
//		right.set(speed);
//		left.set(speed);
		rightspeed = speed;
		leftspeed = speed;
		isvoltagemode = false;
	}
	
	public double getrightposition()
	{
		return right.getPosition();
	}
	public double getleftposition()
	{
		return left.getPosition();
	}
	public void resetPosition()
	{
		right.resetPosition();
		left.resetPosition();
	}
	public double getPosition()
	{
		return (right.getPosition() + left.getPosition()/2);
	}
	public boolean driveToPosition(double pos)
	{
//		right.resetPosition();
//		left.resetPosition();
		
		isvoltagemode = false;
		rightspeed = 200;
		leftspeed = 200;
		if(right.getPosition() > pos)
		{
			rightspeed = 0;
			leftspeed = 0;
			return true;
		}
//		}else if(right.getPosition() < 0 && right.getPosition() < pos)
//		{
//			rightspeed = 0;
//			leftspeed = 0;
//			return true;
//		}
		return false;
	}
	
	public void shiftGear()
	{
		if(getHighGear() == DoubleSolenoid.Value.kForward)
		{
			ptoshift.set(DoubleSolenoid.Value.kReverse);
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		else
		{
			ptoshift.set(DoubleSolenoid.Value.kForward);
			shifter.set(DoubleSolenoid.Value.kForward);
		}
	}
	public DoubleSolenoid.Value getHighGear()
	{
		return shifter.get();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
