package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RotateCommand extends Command {
	PIDController pidcontroller;
	Drive drive;
	double angle;
	double error1;
	double error2;
	double time1;
	double time2;
	Timer timer;
    public RotateCommand(Drive drive, PIDController pidcontroller, double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires (drive); 
    	this.drive = drive;
    	this.pidcontroller = pidcontroller;
    	this.angle = angle;
    	timer = new Timer();
    	timer.start();
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	pidcontroller.setSetpoint(Robot.turnTo(angle));
//    	time1 = timer.get();
    	time1 = Timer.getFPGATimestamp();
    	error1 = pidcontroller.getError();
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	pidcontroller.enable();
    	drive.pidRotate();
    	drive.go();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double currentangle = pidcontroller.getSetpoint() - pidcontroller.getError();
    	time2 = Timer.getFPGATimestamp();
    	error2 = pidcontroller.getError();
    	double errordot =Math.abs((error1 - error2)/(time1 - time2));
    	SmartDashboard.putNumber("errordot", errordot);
    	time1 = time2;
    	error1 = error2;
    	SmartDashboard.putNumber("time 1", time1);
    	SmartDashboard.putNumber("error 1", error1);
    	SmartDashboard.putNumber("time 2", time2);
    	SmartDashboard.putNumber("error 2", error2);
    	SmartDashboard.putNumber("currentangle", currentangle);
    	if(Math.abs(error2) < Parameters.AUTONOMOUS_GYRO_TOLERANCE &&
    			errordot < Parameters.AUTONOMOUS_GYRO_RATE_TOLERANCE)
    	{
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	pidcontroller.disable();
//    	drive.resetPosition();
    	timer.stop();
		drive.go(0,0);
		SmartDashboard.putNumber("isDone?", angle);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
