#include "commands/SwerveControlCmd.h"
#include <frc2/command/Commands.h>
#include <frc2/command/Subsystem.h>
#include "subsystems/Drivetrain.h"
//#include <pathplanner/lib/auto/autoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>



SwerveControlCmd::SwerveControlCmd(Drivetrain& drivetrain, DriverController& Controller, ctre::phoenix6::hardware::Pigeon2& pigeon) 
: subsystem{drivetrain}, 
  DriverControl{Controller},
  IMU{pigeon}
{
  AddRequirements(&subsystem);
}

void SwerveControlCmd::Execute()
{
  auto Speeds = DriverControl.GetChassisSpeeds();
  Speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(Speeds, IMU.GetRotation2d()); // Field Orientated Drive Speeds
  subsystem.SetChassisSpeeds(Speeds);
}

void SwerveControlCmd::End(bool interrupted)
{
  if(!DriverControl.isTranslating().Get())  
  {subsystem.StopAllMotors_();}

}

bool SwerveControlCmd::IsFinished()
{
  return false;
}