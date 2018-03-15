package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2028.robot.Parameters.CanId;
import org.usfirst.frc.team2028.robot.Parameters;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class Drive extends Subsystem implements PIDOutput {
//	private Gyro gyro;
	/** Left side drive sub-sub-system, composition relationship */
	private DriveSide left;
	
	/** Composition relationship to right side drive sub-sub-system */
	private DriveSide right;
	
	/** Pneumantic solenoid to select low or high gear.  Composition relationship */
	private DoubleSolenoid shifter;
	
	private DoubleSolenoid ptoshift;
	
	private Ultrasonic rearultrasonic;
	
	double leftspeed;
	double rightspeed;
	boolean isvoltagemode;
	boolean ispositionmode;
	boolean isauto;
	double position;
	double value_;
	
	private Command defaultCommand;
	
	private XboxController controller;
	
	private Robot robot;
	
	/** 
	 * Default constructor.
	 */
	Drive(Robot rob, XboxController con, Ultrasonic rearultrasonic)
	{
		super();
//		air = new AnalogInput(3);
		shifter = new DoubleSolenoid(Parameters.PNEUMATIC_CHANNEL.LOW_GEAR.getChannel(),
				Parameters.PNEUMATIC_CHANNEL.HIGH_GEAR.getChannel());
		ptoshift = new DoubleSolenoid(Parameters.PNEUMATIC_CHANNEL.PTO_ENGAGE.getChannel(),
				Parameters.PNEUMATIC_CHANNEL.PTO_DISENGAGE.getChannel());
		left = new DriveSide(CanId.LEFT_MASTER.getCanId(), 
				CanId.LEFT_FOLLOWER.getCanId(), 
				Parameters.LEFT_DRIVE_INVERTED, Parameters.LEFT_PHASE, true);
		right = new DriveSide(CanId.RIGHT_MASTER.getCanId(),
				CanId.RIGHT_FOLLOWER.getCanId(), 
				!Parameters.LEFT_DRIVE_INVERTED, Parameters.RIGHT_PHASE, false);
        shifter.set(DoubleSolenoid.Value.kForward);
        controller = con;
        robot = rob;
        this.rearultrasonic = rearultrasonic;
        getDefaultCommand();
	}
	
	
	
	public Command getDefCommand() {
		return defaultCommand;
	}
	
	public double getRightMotorOutputPercent()
	{
		return right.getMotorOutputPercent();
	}
	
	public double getLeftMotorOutputPercent()
	{
		return left.getMotorOutputPercent();
	}
	
	boolean plsstop = false;
	boolean reset = false;
	public boolean rotateToPosition(double angle, double speed)
	{
		SmartDashboard.putNumber("right position", getrightposition());
		SmartDashboard.putNumber("left position", getleftposition());
		double radians = angle * (Math.PI/180);
		double distance = (radians*Parameters.ROBOT_RADIUS)*1.153;
		if(reset == false)
		{
			resetPosition();
			reset = true;
		}
		if(angle > 0 && plsstop == false)
		{
			rightspeed = -speed;
			leftspeed = speed;
			isvoltagemode = true;
			return false;
		}
		else if (angle < 0 && plsstop == false)
		{
			leftspeed = -speed;
			rightspeed = speed;
			isvoltagemode = true;
			return false;
		}
		if(Math.abs(getrightposition()) > distance || 
				Math.abs(getleftposition()) > distance)
		{
			stop();
			plsstop = true;
			return true;
		}
		else
		{
			plsstop = false;
			return false;
		}
	}
	
	public void resetRotatePosition()
	{
		reset = false;
	}
	
	/**
	 * Retrieve the motor output current for the left drive side 
	 * and right drive side and returns the greatest value
	 * @return double - greatest current value of right and left side.
	 */
	public double getMaximumMotorCurrent()
	{
		return Math.max(right.getMaximumMotorCurrent(), left.getMaximumMotorCurrent());
	}
	
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

	public double getRightVelocity()
	{
		return right.getVelocity();
	}
	public double getLeftVelocity()
	{
		return left.getVelocity();
	}
	
	public void driveFollow(double speed)
	{
//		right.set(-speed);
//		left.set(-(speed-value_));
		
		rightspeed = speed;
		leftspeed = (speed-value_);
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
		rightspeed = value_;
		leftspeed = -value_;
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
	
//	public double getAirPressure()
//	{
//		return air.getAverageVoltage();
//	}
	
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

	public void gameDrive(double leftSpeed, double turnSpeed)
	{
		if(leftSpeed != 0){
		leftspeed = leftSpeed + (leftSpeed*turnSpeed);
		rightspeed = leftSpeed - (leftSpeed*turnSpeed);
		}
		else
		{
			leftspeed = turnSpeed*0.9;
			rightspeed = -turnSpeed*0.9;
		}
		isvoltagemode = true;
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
	double startposition;
	double test = 0;
	public boolean driveToPosition(double pos)
	{
		if (test == 0)
		{
			startposition = right.getPosition();
			test +=1;
		}
//		right.resetPosition();
//		left.resetPosition();
		double difference = (right.getPosition() - pos);
		SmartDashboard.putNumber("Starting position", startposition);
		SmartDashboard.putNumber("POS", pos);
		isvoltagemode = false;
		double direction = Math.signum(pos - startposition);
		rightspeed = 300 * direction;
		leftspeed = 300 * direction;
		SmartDashboard.putNumber("direction", direction);
		SmartDashboard.putNumber("Hello", 0);
		SmartDashboard.putNumber("Right Position", right.getPosition());
		SmartDashboard.putNumber("pos - right.getPosition", pos - right.getPosition());
//		if(pos - right.getPosition() < direction*0.3)
//		{
//			SmartDashboard.putNumber("Hello", 1);
//			rightspeed = 0;
//			leftspeed = 0;
//			test = 0;
//			return true;
//		}
		if(direction < 0 && right.getPosition() < pos)
		{
			rightspeed = 0;
			leftspeed = 0;
			test = 0;
			return true;
		}
		else if(direction > 0 && right.getPosition() > pos)
		{
			rightspeed = 0;
			leftspeed = 0;
			test = 0;
			return true;
		}
		return false;
	}
	
	public void resetDriveToPositions()
	{
		test = 0;
	}
	
	public boolean driveForward(double pos)
	{
		if (test == 0)
		{
			startposition = right.getPosition();
			test +=1;
		}
		double stop = startposition+pos;
		rightspeed = Parameters.AUTO_RIGHT_DRIVE_FORWARD_SPEED;
		leftspeed = Parameters.AUTO_LEFT_DRIVE_FORWARD_SPEED;
		isvoltagemode = false;
		if(right.getPosition() > stop)
		{
			rightspeed = 0;
			leftspeed = 0;
			return true;
		}
		return false;
	}
 
	public boolean driveReverse(double pos)
	{
		if(test == 0)
		{
			startposition = right.getPosition();
			test +=1;
		}
		double stop = startposition - pos;
		rightspeed = Parameters.AUTO_RIGHT_DRIVE_REVERSE_SPEED;
		leftspeed = Parameters.AUTO_LEFT_DRIVE_REVERSE_SPEED;
		isvoltagemode = false;
		if(right.getPosition() < stop)
		{
			rightspeed = 0;
			leftspeed = 0;
			return true;
		}
		return false;
	}
	public void shiftGear()
	{
		if(getHighGear() == DoubleSolenoid.Value.kForward)
		{
//			ptoshift.set(DoubleSolenoid.Value.kReverse);
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		else
		{
//			ptoshift.set(DoubleSolenoid.Value.kForward);
			shifter.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	public void setLowGear()
	{
		shifter.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void setHighGear()
	{
		shifter.set(DoubleSolenoid.Value.kForward);
	}
	
	public void shiftPTO()
	{
		if(getPTOHigh() == DoubleSolenoid.Value.kForward)
		{
			ptoshift.set(DoubleSolenoid.Value.kReverse);
		}
		else
		{
			ptoshift.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	public void setPTOHigh()
	{
		ptoshift.set(DoubleSolenoid.Value.kForward);
	}
	
	public void setPTOLow()
	{
		ptoshift.set(DoubleSolenoid.Value.kReverse);
	}
	
	public DoubleSolenoid.Value getHighGear()
	{
		return shifter.get();
	}
	public DoubleSolenoid.Value getPTOHigh()
	{
		return ptoshift.get();
	}

	@Override
	protected void initDefaultCommand() {
//		System.out.println("\tInitizliaing default drive command");
//		defaultCommand = new JoystickDrive(controller, this, robot, rearultrasonic);
//		this.setDefaultCommand(defaultCommand);
	}
}
