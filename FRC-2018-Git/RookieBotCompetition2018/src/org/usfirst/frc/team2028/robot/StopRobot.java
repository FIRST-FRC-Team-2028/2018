package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StopRobot extends Command {

	Drive drive;
    public StopRobot(Drive drive) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(drive);
    	this.drive = drive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(0.2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.stop();
    	drive.go();
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
