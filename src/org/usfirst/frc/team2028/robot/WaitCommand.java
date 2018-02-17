package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 	This Command waits for a specified time given in its constructor parameters.
 * 
 * @author robotics
 */
public class WaitCommand extends Command {
	
	double settime;
	
	/**
	 * 
	 * @param time_
	 */
    public WaitCommand(double time_) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	settime = time_;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	this.setTimeout(settime);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Intentionally does nothing.
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	
    }
}
