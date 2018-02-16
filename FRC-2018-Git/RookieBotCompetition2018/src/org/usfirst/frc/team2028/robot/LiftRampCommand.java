package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftRampCommand extends Command {
	Ramp ramp;
	Drive drive;
    public LiftRampCommand(Ramp ramp, Drive drive) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(ramp);
    	requires(drive);
    	this.ramp = ramp;
    	this.drive = drive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	ramp.deployOutriggers();
    	drive.setPTOLow();
    	this.setTimeout(7);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.go();
    	drive.set(50);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.isTimedOut();
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
