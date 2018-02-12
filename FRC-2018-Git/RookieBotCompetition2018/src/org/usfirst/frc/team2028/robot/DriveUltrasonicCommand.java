package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveUltrasonicCommand extends Command {
	Drive drive;
	Ultrasonic ultrasonic;
	double distance;
    public DriveUltrasonicCommand(Drive drive, Ultrasonic ultrasonic, double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.ultrasonic = ultrasonic;
    	this.drive = drive;
    	this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("ultra distance", ultrasonic.getDistance());
    	drive.go();
    	if(ultrasonic.getDistance()  < 30)
    	{
    		drive.set(-150);
        	SmartDashboard.putNumber("ultra distance", ultrasonic.getDistance());
        	
    	}
    	else
    	{
    		drive.set(-300);
        	SmartDashboard.putNumber("ultra distance", ultrasonic.getDistance());

    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	SmartDashboard.putNumber("ultra distance", ultrasonic.getDistance());
    	if(ultrasonic.getDistance() < distance)
    	{
    		return true;
    	}
    	else{
        return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
//    	drive.go();
    	drive.stop();
    	drive.go();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
