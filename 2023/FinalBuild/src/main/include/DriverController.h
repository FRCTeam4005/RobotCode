

#ifndef DRIVER_CONTROLLER_H
#define DRIVER_CONTROLLER_H

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <math.h>

#include "Drivetrain.h"
#include <units/angular_velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

class DriverController
{
public:

	DriverController(DriverController &other) = delete;	
	DriverController(DriverController &&other) = delete;
	void operator=(const DriverController &) = delete;
	void operator=(const DriverController &&) = delete;

	static DriverController& GetInstance()
	{
		static DriverController Instance(DriverControllerPort);
		return Instance;
	}

	DriverController(int port) 
	{
		Controller = frc::XboxController{port};
	}

	units::meters_per_second_t getFWD()
	{
		auto ControllerValue(FwdFilter.Calculate(frc::ApplyDeadband(Controller.GetLeftY(), 0.035)));

		if(Get_Lower_Gear_Button())
		{
			return ControllerValue * (Capped_FWD_Speed / units::dimensionless_t{2});
		}
		else if(Get_Higher_Gear_Button())
		{
			return ControllerValue * Max_FWD_Speed;
		}

		return ControllerValue * Capped_FWD_Speed;
	}

	units::meters_per_second_t getSTR()
	{
		auto ControllerValue(-StrFilter.Calculate(frc::ApplyDeadband(Controller.GetLeftX(), 0.035)));

		if(Get_Lower_Gear_Button())
		{
			return ControllerValue * (Capped_FWD_Speed / units::dimensionless_t{2});
		}
		else if(Get_Higher_Gear_Button())
		{
			return ControllerValue * Max_FWD_Speed;
		}
		return ControllerValue * Capped_FWD_Speed;
	}

	units::radians_per_second_t getRCW()
	{
		return  -RcwFilter.Calculate(frc::ApplyDeadband(Controller.GetRightX(), 0.035)) * Capped_RCW_Speed * 2;// multiply by two because the  "ChassisSpeeds speeds(fwd2, str2, RCWPIDVEL)" always halves the rotational speed for some reason
	}

	bool Get_HandBreak_Button()
	{
		return Controller.GetBButton();
	}
	
	bool Get_Lower_Gear_Button()
	{
		if((abs(Controller.GetLeftTriggerAxis()) >= .5))
		{
			return true;
		}
		return false;
	}

	bool Get_Higher_Gear_Button()
	{
		if((abs(Controller.GetRightTriggerAxis()) >= .5))
		{
			return true;
		}
		return false;
	}

	bool Get_RGB_Toggle_Button()
	{
		static bool hasRan = false;
		if(Controller.GetAButton() && !hasRan)
		{
			hasRan = true;
			return true;
		}
		else if(hasRan && !Controller.GetAButton())
		{
			hasRan = false;
			return false;
		}
		else if(hasRan && Controller.GetAButton())
		{
			hasRan = true;
			return false;
		}
		return false;
	}

	units::degree_t Get_Dpad_Degree()
	{
		int test = Controller.GetPOV();

		return units::degree_t(test);
	}

private:
	frc::XboxController Controller{0};
	frc::SlewRateLimiter<units::scalar> FwdFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> StrFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> RcwFilter{3 / 1_s};
};

#endif