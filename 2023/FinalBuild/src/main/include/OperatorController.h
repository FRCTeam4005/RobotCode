

#ifndef OPERATOR_CONTROLLER_H
#define OPERATOR_CONTROLLER_H

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotConstants.h"

class OperatorController
{
public:
	OperatorController(OperatorController &other) = delete;
	OperatorController(OperatorController &&other) = delete;
	void operator=(const OperatorController &) = delete;
	void operator=(const OperatorController &&) = delete;

	static OperatorController& GetInstance()
	{
		static OperatorController Instance(OperatorControllerPort);
		return Instance;
	}

	typedef enum{
	NOT_SET_4ba_PRESET,
	LOWEST_4ba_PRESET,
	MIDDLE_4ba_PRESET,
	HIGHEST_4ba_PRESET
	} FourbaPreset_t;


	
	double getRightTrigger()
	{
		return Controller.GetRightTriggerAxis();
	}

	double getLeftTrigger()
	{
		return Controller.GetLeftTriggerAxis();
	}

	double getCarriageMovement()
	{
		return -Controller.GetLeftY();
	}

	double Get4BAUpDown()//Right analog stick controls the motor that moves the 4 bar arm veritcal position
	{
		return -Controller.GetRightY();
	}

	bool getClawExtend()
	{
		return Controller.GetLeftBumper();
	}

	bool getClawRetract()
	{
		return Controller.GetRightBumper();
	}

	bool getGrabberButton()
	{
		return Controller.GetYButton();
	}

	bool getBButton()
	{
		return Controller.GetBButton();
	}

	bool getAButton()
	{
		return Controller.GetAButton();
	}

	bool getXButton()
	{
		return Controller.GetXButton();
	}


	FourbaPreset_t FourbaPositionSelect()
	{
		if(Get_4baTopExtended())
		{
			return HIGHEST_4ba_PRESET;
		}
		else if(get_4baMidposition())
		{
			return MIDDLE_4ba_PRESET;
		}
		else if(get_4baBottomPulledIn())
		{
			return LOWEST_4ba_PRESET;
		}
		else
		{
			return NOT_SET_4ba_PRESET;
		}
	}

private:
	frc::XboxController Controller{0};
	frc::SlewRateLimiter<units::scalar> FwdFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> StrFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> RcwFilter{3 / 1_s};

	OperatorController(int port) 
	{
		Controller = frc::XboxController{port};
	}

	bool Get_4baTopExtended()//pushes cariage out, extends the claw(does not open claw), and lowers 4bar into lowest position
	{
		return ((Controller.GetPOV() >= 315 || Controller.GetPOV() <= 45) && Controller.GetPOV() >= 0) ;
	}

	bool get_4baBottomPulledIn()// pulls the carriage in, retracts the claw(does not open claw), and lowers 4bar into lowest position
	{
		return (Controller.GetPOV() >= 135 && Controller.GetPOV() <= 225);
	}

	bool get_4baMidposition()//pushes the carriage out, extends the claw(does not open claw), and raises or lowers 4bar to mid position
	{
		return (Controller.GetPOV() >= 225 && Controller.GetPOV() <= 315);
	}

};

#endif