package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

/**
 *
 */
public class JoystickDrive extends Command {

	private XboxController Controller;
	
	private Drive drive;
	
	private double rightSpeed;
	
	private double leftSpeed;
	
	private Robot robot;
	
	private Ultrasonic rearultrasonic;
	
    public JoystickDrive(XboxController con, Drive drive, Robot robot, Ultrasonic rearultrasonic) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(drive);
    	Controller = con;
    	this.robot = robot;
    	this.drive = drive;
    	this.rearultrasonic = rearultrasonic;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
		double leftStickSpeed = Controller.getRawAxis(1) * -4069 * 500 / 600;
		double rightStickSpeed = Controller.getRawAxis(5) * -4069 * 500 / 600;
    	
		drive.go();
		
		if (Math.abs(Controller.getRawAxis(1)) > .1)
		{
			rightSpeed = -Controller.getRawAxis(1);
		}
		else
		{
			rightSpeed = 0;
		}
		if (Math.abs(Controller.getRawAxis(5)) > .1)
		{
			leftSpeed = -Controller.getRawAxis(5);
			leftStickSpeed = Controller.getRawAxis(5) * -4069 * 500 / 600;
		}
		else
		{
			leftSpeed = 0;
			leftStickSpeed = 0;
		}
		
		if((Controller.getRawButton(6) && !robot.isTurning()) || 
				(Controller.getRawButton(6) && !robot.isFollowingCube()))
		{
			SmartDashboard.putNumber("loopValue", 1);
			drive.rotate(leftStickSpeed);
		} 
		else if ((Controller.getRawButton(5) && !robot.isTurning()) || 
				(Controller.getRawButton(5) && !robot.isFollowingCube())) 
		{
			SmartDashboard.putNumber("loopValue", 2);
			SmartDashboard.putNumber("left side velocity", -drive.getRightVelocity());
			SmartDashboard.putNumber("right side velocity", -drive.getLeftVelocity());
//			drive.resetPosition();
			if(Controller.getRawButtonPressed(5))
			{
				drive.resetDriveToPositions();
			}
			drive.driveReverse(160);
//			drive.set(50);
		}
		else if(!robot.isTurning() && !robot.isFollowingCube())
		{
			SmartDashboard.putNumber("loopValue", 3);
			drive.setVoltage(leftSpeed, rightSpeed);
		}
		else if(Controller.getRawAxis(3) > 0.5 && !robot.isFollowingCube() ||
				Controller.getRawAxis(3) > 0.5 && !robot.isTurning())
		{
			drive.set(rearultrasonic.speed());
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
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
