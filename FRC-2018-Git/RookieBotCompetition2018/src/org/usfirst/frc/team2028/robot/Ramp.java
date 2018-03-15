package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Ramp extends Subsystem {

	private Solenoid solenoid;
	
	Ramp()
	{
		solenoid = new Solenoid(Parameters.PNEUMATIC_CHANNEL.RAMP_DEPLOY.getChannel());
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public void deployRamps()
	{
		solenoid.set(true);
	}
	public void openGeorge()
	{
		solenoid.set(false);
	}
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

