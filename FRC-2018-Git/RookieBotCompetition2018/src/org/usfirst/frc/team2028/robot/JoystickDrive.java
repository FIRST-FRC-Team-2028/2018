package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 *
 */
public class JoystickDrive extends Command {

	private XboxController Controller;
	
	private Drive drive;
	
	private double rightSpeed;
	
	private double leftSpeed;
	
	private double turnSpeed;
	
	private Robot robot;
	
	private Ultrasonic rearultrasonic;
	
	double maxRate;
	
	double beforeleft = 0;
	
	double beforeright = 0;
	
	double timebefore = 0;
	
	Timer timer = new Timer();
	
    public JoystickDrive(XboxController con, Drive drive, Robot robot, Ultrasonic rearultrasonic) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(drive);
    	Controller = con;
    	this.robot = robot;
    	this.drive = drive;
    	this.rearultrasonic = rearultrasonic;
    	maxRate = 1;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timebefore = timer.getFPGATimestamp();
    }

    public double[] smooth(double leftcurrent, double rightcurrent, double timecurrent)
    {
    	double Lsize = Math.min(Math.abs(maxRate*(timecurrent-timebefore)+beforeleft), Math.abs(leftcurrent));
        double Lafter = Math.copySign(Lsize, leftcurrent);
    	double Rsize = Math.min(Math.abs(maxRate*(timecurrent-timebefore)+beforeleft), Math.abs(rightcurrent));
        double Rafter = Math.copySign(Rsize, rightcurrent);
        SmartDashboard.putString("left t0,t1,j0,j1,after:"," "+timebefore+" "+timecurrent+" "+beforeleft+" "+leftcurrent);
        SmartDashboard.putString("right t0,t1,j0,j1,after:"," "+timebefore+" "+timecurrent+" "+beforeright+" "+rightcurrent);
    	beforeleft = Lafter;
    	beforeright = Rafter;
    	timebefore = timecurrent;
    	double[] afters = {Lafter, Rafter};
    	return afters;
    }
    public double leftsmooth(double current, double timecurrent)
    {
    	double size = Math.min(Math.abs(maxRate*(timecurrent-timebefore)+beforeleft), Math.abs(current));
        double after = Math.copySign(size, current);
        SmartDashboard.putString("left t0,t1,j0,j1,after:"," "+timebefore+" "+timecurrent+" "+beforeleft+" "+current);
    	beforeleft = after;
    	timebefore = timecurrent;
    	return after;
    }
    
    public double rightsmooth(double current, double timecurrent)
    {
    	double size = Math.min(Math.abs(maxRate*(timecurrent-timebefore)+beforeright), Math.abs(current));
        double after = Math.copySign(size, current);
        SmartDashboard.putString("right t0,t1,j0,j1,after:"," "+timebefore+" "+timecurrent+" "+beforeleft+" "+current);
        beforeright = after;
    	timebefore = timecurrent;
    	return after;
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	double time = timer.getFPGATimestamp();
		double rightStickSpeed = Controller.getRawAxis(1) * -4069 * 500 / 600;
		double leftStickSpeed  = Controller.getRawAxis(5) * -4069 * 500 / 600;
    	
		
		
		drive.go();
		
		if (Math.abs(Controller.getRawAxis(1)) > .1)
		{
			rightSpeed = -(Controller.getRawAxis(1));
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
		if (Math.abs(Controller.getRawAxis(4)) > .1)
		{
			turnSpeed = Controller.getRawAxis(4);
		}
		else
		{
			turnSpeed = 0;
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
//			if(Parameters.VIDEO_GAME_DRIVE_MODE)
//			{
//				drive.gameDrive(leftSpeed, turnSpeed);
//			}
//			else{
//			double[] smooths = smooth(leftSpeed, rightSpeed, time);
//			drive.setVoltage(smooths[0], smooths[1]);
				drive.setVoltage(leftSpeed, rightSpeed);
//			}			
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
