
#define IsDriving speeds.omega.value() != 0 || speeds.vx.value() != 0 || speeds.vy.value() != 0 

#include "Drivetrain.h"
#include "Pigeon.h"
#include <frc/MathUtil.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/dimensionless.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>
#include <rev/SparkPIDController.h>
#include <frc/controller/PIDController.h>
#include <math.h>
#include <RobotConstants.h>

#include <frc/Timer.h>

using namespace ctre::phoenix6;

frc::Timer DriveTrainTimer;



Drivetrain::Drivetrain()
{
  configs::CANcoderConfiguration  frMagnet{},flMagnet{},brMagnet{},blMagnet{};

  frMagnet.MagnetSensor.MagnetOffset = 227.900391/360;
  frMagnet.MagnetSensor.SensorDirection = true;
  // frMagnet.MagnetSensor.AbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);
  frModule.Turn_Encoder->GetConfigurator().Apply(frMagnet);

  flMagnet.MagnetSensor.MagnetOffset = 272.197266/360;
  flMagnet.MagnetSensor.SensorDirection = true;
  // flMagnet.MagnetSensor.AbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);
  flModule.Turn_Encoder->GetConfigurator().Apply(flMagnet);

  brMagnet.MagnetSensor.MagnetOffset = 12.304688/360;
  brMagnet.MagnetSensor.SensorDirection = true;
  // brMagnet.MagnetSensor.AbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);
  brModule.Turn_Encoder->GetConfigurator().Apply(brMagnet);

  blMagnet.MagnetSensor.MagnetOffset = 218.0566411/360;
  blMagnet.MagnetSensor.SensorDirection = true;
  // blMagnet.MagnetSensor.AbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);
  blModule.Turn_Encoder->GetConfigurator().Apply(blMagnet);


  Pos_RCWpid.EnableContinuousInput(0,360);
  Pos_RCWpid.SetTolerance(1.5,2);

}

/**
 * @brief singlton: https://refactoring.guru/design-patterns/singleton
 * a singleton is used so that multiple objects are not controlling the same things. 
 * Stops multiple instances of a object from existing
*/
Drivetrain& Drivetrain::getInstance()
{
  static Drivetrain Instance;
  return Instance;
}

void Drivetrain::fieldCentricSwerveCalculation(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw)
{
  auto pData = Pigeon::GetInstance().GetPigeonData();
  units::radian_t angle = -(pData.FusionAngle + 180_deg);
  //units::degree_t angle_deg = -(pData.FusionAngle + 180_deg); this is just for printing

  units::meters_per_second_t fwd2 = (fwd * cos(angle.value()) + str * sin(angle.value()));
  units::meters_per_second_t str2 = (-fwd * sin(angle.value()) + str * cos(angle.value()));

  swerveCalculation(fwd2,str2,rcw);
}

void Drivetrain::swerveCalculation(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw)
{
  frc::ChassisSpeeds speeds(fwd, str, RotationPidCalculatione(rcw));
  auto moduleStates = m_Kinematics.ToSwerveModuleStates(speeds);

  flState = moduleStates[0];
  frState = moduleStates[1];
  blState = moduleStates[2];
  brState = moduleStates[3];

  if(IsDriving)
  {
    SetAllModuleStates();
  }
  else
  {
    ZeroAllMotors();
  }
}

/**
 * @param DC the Driver Controller
 * @brief takes in a controlller input and uses it to control the robot. The controls used are fwd, str, rcw and a brake.
 * 
*/
void Drivetrain::teleOpSwerve(DriverController &DC)
{

  auto fwd = DC.getFWD();
  auto str = DC.getSTR();
  auto rot = DC.getRCW();

  if(DC.Get_HandBreak_Button())
  {
    ParkingBrake();
  }
  else
  {
    fieldCentricSwerveCalculation(fwd,str,rot);
  }
}

void Drivetrain::autoSwerve(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw)
{
  fieldCentricSwerveCalculation(fwd,str,rcw);
}

/**
 *@brief The wheels will turn so that they form a X and the drive motors will have no velocity
*/
void Drivetrain::ParkingBrake()
{
  flState.speed = 0_mps;
  frState.speed = 0_mps;
  blState.speed = 0_mps;
  brState.speed = 0_mps;

  flState.angle = -45_deg;
  frState.angle = 45_deg;
  blState.angle = 45_deg;
  brState.angle = -45_deg;

  SetAllModuleStates();
}

/**
 * @brief if the function is called the max speed of the drive motors is divided by a scalar value. tldr: robot goes slower
*/
void Drivetrain::LowerGearDrive()
{
  flState.speed /= LowerGearDivider;
  frState.speed /= LowerGearDivider;
  blState.speed /= LowerGearDivider;
  brState.speed /= LowerGearDivider;

  SetAllModuleStates();
}

/**
 * @brief takes the new state of all the modules and applies the states to the motors/modules.
*/
void Drivetrain::SetAllModuleStates()
{
  frModule.Set_State(frState);
  flModule.Set_State(flState);
  brModule.Set_State(brState);
  blModule.Set_State(blState);
}

/**
 * @brief the turn motors align; so, that Drive Motors face forwards
*/
void Drivetrain::AlignAllMotors()
{
  flModule.Allign_Motor();
  frModule.Allign_Motor();
  blModule.Allign_Motor();
  brModule.Allign_Motor();
}

/**
 * @brief all the motors (Drive and Turn) spin freely instead of being powered
*/
void Drivetrain::ZeroAllMotors()
{
  flModule.Go_Limp();
  frModule.Go_Limp();
  blModule.Go_Limp();
  brModule.Go_Limp();
}

void Drivetrain::PrintEncoders()
{
  frc::SmartDashboard::PutNumber("fl encoder value", flModule.Turn_Encoder->GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("fr encoder value", frModule.Turn_Encoder->GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("bl encoder value", blModule.Turn_Encoder->GetAbsolutePosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("br encoder value", brModule.Turn_Encoder->GetAbsolutePosition().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("fl speed ", flModule.Drive_Encoder->GetVelocity());
  frc::SmartDashboard::PutNumber("fr speed ", frModule.Drive_Encoder->GetVelocity());
  frc::SmartDashboard::PutNumber("bl speed ", blModule.Drive_Encoder->GetVelocity());
  frc::SmartDashboard::PutNumber("br speed ", brModule.Drive_Encoder->GetVelocity()); 
}

frc::PIDController Rotational_vel_pid{1,.64,.04,10_ms};

units::degrees_per_second_t Drivetrain::RotationPidCalculatione(units::degrees_per_second_t desiredVelocity)
{
  units::degrees_per_second_t RCWPIDVEL = 0_deg_per_s;

  if(desiredVelocity.value() != 0)
  {
    RCWPIDVEL = units::degrees_per_second_t(Rotational_vel_pid.Calculate(Pigeon::GetInstance().GetYawRate().value(), desiredVelocity.value()));
  }
  return RCWPIDVEL;
}


