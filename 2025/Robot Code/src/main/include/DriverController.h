#pragma once

#ifndef DRIVER_CONTROLLER_H
#define DRIVER_CONTROLLER_H

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <math.h>
#include "subsystems/Drivetrain.h"

#include <units/angular_velocity.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include<frc2/command/Command.h>

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>


#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "subsystems/Claw.h"
#include "subsystems/Elevator.h"
#include "subsystems/Algae.h"


using namespace SwerveDriveConstants;
using namespace OIConstants;

class DriverController
{
public:

	DriverController(int port) 
	{
		Controller = std::make_unique<frc2::CommandXboxController>(port);
	}

	frc::ChassisSpeeds GetChassisSpeeds()
	{
		return frc::ChassisSpeeds
		{
			-GetDBVal(Controller->GetLeftY()) * Capped_FWD_Speed,
			-GetDBVal(Controller->GetLeftX()) * Capped_STR_Speed ,
			-frc::ApplyDeadband(Controller->GetRightX(),0.05) * Capped_RCW_Speed
		};
	}

	frc2::CommandPtr setRumble(double power){
		return frc2::cmd::RunOnce([this, power] {Controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble, power);});
	}

	frc2::Trigger isTranslating() 
	{
		return frc2::Trigger([this] {
			auto speeds = GetChassisSpeeds();
			auto retval = !(speeds.vx.value() == 0) || !(speeds.vy.value() == 0);
			frc::SmartDashboard::PutBoolean("isTranslating:", retval);
			return retval;
		});
	}

	frc2::Trigger isRotating() 
	{
		return frc2::Trigger([this] {
			auto speeds = GetChassisSpeeds();
			auto retval = (speeds.omega.value() != 0);
			frc::SmartDashboard::PutBoolean("isRotating:", retval);
			return retval;
		});
	}
	
	frc2::Trigger Get_Lower_Gear_Button()
	{
		return frc2::Trigger([this] {return abs(Controller->GetLeftTriggerAxis()) > .5; });
	}

	frc2::Trigger Get_Higher_Gear_Button()
	{
		return frc2::Trigger([this] {return abs(Controller->GetRightTriggerAxis()) > .5; });
	}

	double getfwd()
	{
		return  Controller->GetLeftY();
	}

	std::unique_ptr<frc2::CommandXboxController> Controller;

private:
	double GetDBVal(double value)
	{
		return frc::ApplyDeadband(value, 0.04);
	}
};	

#endif