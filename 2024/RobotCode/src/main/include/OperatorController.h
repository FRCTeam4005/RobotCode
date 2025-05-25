#pragma once

#ifndef OPERATOR_CONTROLLER_H
#define OPERATOR_CONTROLLER_H

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
#include <iostream>
#include <unistd.h>
#include <string>




class OperatorController
{
public:

	OperatorController(int port) 
	{
		Controller = frc2::CommandXboxController{port};
	}

  frc2::Trigger Get_TopDPad(){
    return frc2::Trigger([this]{return (((Controller.GetPOV() >= 315 || Controller.GetPOV() <=45) && (Controller.GetPOV() >= 0)));});
  }

  frc2::CommandPtr setRumble(double power){
    printf("Settting rumble to %lf", power);
    std::cout<<"Settting rumble to %lf"<<power;
		return frc2::cmd::RunOnce([this, power] {Controller.SetRumble(frc::GenericHID::RumbleType::kBothRumble, power);});
	}
	// frc2::Trigger intakeNote()
	// {
	// 	return Controller.A();
	// }

	frc2::CommandXboxController Controller{0};
private:
	frc::SlewRateLimiter<units::scalar> FwdFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> StrFilter{3 / 1_s};
	frc::SlewRateLimiter<units::scalar> RcwFilter{3 / 1_s};

};

#endif