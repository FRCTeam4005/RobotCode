package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OperatorController 
{
    private final XboxController operatorControl;

    public OperatorController(int port)
	{
		operatorControl = new XboxController(port);
	}

	public double getRightStickYAxis()
	{
		return operatorControl.getRightY();
	}

	public double getLeftStickYAxis()
	{
		return -operatorControl.getLeftY();
	}

	public double getLeftTrigger()
	{
		return operatorControl.getLeftTriggerAxis();
	}

	public Boolean getyButton()
	{
		return operatorControl.getYButton();
	}

	public Boolean getAButton()
	{
		return operatorControl.getAButton();
	}

	public boolean getBButton()
	{
		return (operatorControl.getBButton());
	}

	public boolean getXButton()
	{
		return (operatorControl.getXButton());
	}

	public double getRightTriggerButton()
	{
		return operatorControl.getRightTriggerAxis();
	}

	public boolean getCenterUp()
	{
		return operatorControl.getRightBumper();
	}

	public boolean getCenterDown()
	{
		return operatorControl.getLeftBumper();
	}

	public void rumble(boolean if_rumble)
	{
		if(if_rumble)
		{
		operatorControl.setRumble(RumbleType.kLeftRumble, 1);
		operatorControl.setRumble(RumbleType.kRightRumble, 1);
		}
		else
		{
			operatorControl.setRumble(RumbleType.kLeftRumble, 0);
			operatorControl.setRumble(RumbleType.kRightRumble, 0);
		}
	}


	
}
