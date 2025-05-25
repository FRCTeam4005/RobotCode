package frc.robot.controllers;

public class OperatorController
{
	private final XboxController operatorControl;

	public OperatorController(int port)
	{
		operatorControl = new XboxController(port);
	}

	public double getIntake()
	{
		return operatorControl.getRightStickYAxis();
	}

	public double getConveyor()
	{
		return -operatorControl.getLeftStickYAxis();
	}

	public boolean getShoot()
	{
		return operatorControl.getAButton();
	}

	public Boolean getLift()
	{
		return operatorControl.getBButton();
	}

	public Boolean getFeeder()
	{
		return operatorControl.getXButton();
	}

	public boolean getCenterUp()
	{
		return operatorControl.getRightBumper();
	}

	public boolean getCenterDown()
	{
		return operatorControl.getLeftBumper();
	}

	public boolean getAutoAim()
	{
		return (operatorControl.getRightTriggerAxis() > 0.5);
	}
}