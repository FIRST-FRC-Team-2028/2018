package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * drive robot a set distance
 */
public class DriveCommand extends Command {
	Drive drive;
	double pos;
	double startposition;
	double leftstartpos;
	double initialright;
	double initialleft;
    public DriveCommand(Drive drive_, double pos_) {
        /** 
         * Drive robot a specified distance
         */
    	// Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	drive = drive_;
    	requires(drive);
    	pos = pos_;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	drive.resetPosition();
    	startposition = drive.getrightposition();
    	SmartDashboard.putNumber("initializedleftPosition", drive.getleftposition());
    	SmartDashboard.putNumber("initializedrightPosition", drive.getrightposition());
    	drive.driveToPosition(startposition+pos);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	drive.go();    	
    	SmartDashboard.putNumber("setPosition", pos);
    	SmartDashboard.putNumber("ExecuteleftPosition", drive.getleftposition());
    	SmartDashboard.putNumber("ExecuterightPosition", drive.getrightposition());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	
        return drive.driveToPosition(startposition+pos);
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.stop();
    	drive.go();
//    	drive.resetPosition();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
