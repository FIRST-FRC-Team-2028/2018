package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 *
 */
public class TimedRotateCommand extends Command {
	double angle;
	double speed;
	Command rotate;
	double anglerate;
	Drive drive;
	public TimedRotateCommand(double speed, double angle, Drive drive) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(drive);
		this.drive = drive;
		this.angle = angle;
		this.anglerate = angleRate(speed);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double time = angle/anglerate;
		setTimeout(time);
	}

	protected double angleRate(double speed)
	{
		return 0.1*speed; //m (speed) + b;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		drive.rotate(speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		drive.stop();
		drive.go();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
