package frc.robot.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriverController
{
  private final XboxController driverControl;
  private SlewRateLimiter fwdFilter = new SlewRateLimiter(1.5);
  private SlewRateLimiter strFilter = new SlewRateLimiter(1.5);
  private SlewRateLimiter rcwFilter = new SlewRateLimiter(1.5);

  public DriverController(int port)
  {
    driverControl = new XboxController(port);
  }

  public double getFWD()
  {
    return fwdFilter.calculate(driverControl.getLeftStickYAxis());
  }

  public double getSTR()
  {
    return strFilter.calculate(driverControl.getLeftStickXAxis());
  }

  public double getRCW()
  {
    return rcwFilter.calculate(driverControl.getRightStickXAxis());
  }

  public boolean getYButton()
  {
    return driverControl.getYButton();
  }

  public boolean getToggleFieldCentric()
	{
		return driverControl.getStartButton();
	}
}