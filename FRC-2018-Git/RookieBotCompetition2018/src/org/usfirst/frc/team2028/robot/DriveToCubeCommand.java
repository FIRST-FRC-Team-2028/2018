package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 * from side of switch get cube from power cube zone
 */
public class DriveToCubeCommand extends Command {
	Drive drive;
	Ultrasonic ultrasonic;
	PIDController cameraController;
	Gripper gripper;
    public DriveToCubeCommand(Drive drive, Ultrasonic ultrasonic, PIDController cameraController, Gripper gripper){
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.drive = drive;
    	this.ultrasonic = ultrasonic;
    	this.cameraController = cameraController;
    	this.gripper = gripper;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	cameraController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.driveFollow(ultrasonic.speed());
    	gripper.pickupCube();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(gripper.isCubeHeld())
    	{
    		cameraController.disable();
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
