
#include "Pigeon.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/CoordinateAxis.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>
#include <ctre/phoenix/platform/Platform.hpp>

using namespace ctre::phoenix6::hardware;

Pigeon::Pigeon()
{

  RobotPigeon.SetFusedHeading(0.0);
  RollOffset = -units::degree_t{RobotPigeon.GetRoll()};

}

Pigeon& Pigeon::GetInstance()
{
  static Pigeon Instance;
  return Instance;
}

PigeonData Pigeon::GetPigeonData()
{

  PigeonIMU::FusionStatus fusionStatus;
  double xyz_degrees_per_s[3];
  RobotPigeon.GetRawGyro(xyz_degrees_per_s);
  RobotPigeon.GetFusedHeading(fusionStatus);
  units::degree_t fusionDegrees{fusionStatus.heading};

  return PigeonData( fusionDegrees, 
                     xyz_degrees_per_s[2],
                     (RobotPigeon.GetState() == PigeonIMU::PigeonState::Ready));
}

units::degrees_per_second_t Pigeon::GetYawRate()
{
  double xyz[3];
  RobotPigeon.GetRawGyro(xyz);
  return units::degrees_per_second_t{xyz[2]};
}

units::degree_t Pigeon::GetRoll()
{
  static double alpha = 0.75;

  RollFiltered = (RobotPigeon.GetRoll() * alpha) + (RollFiltered * (1 - alpha));

  return units::degree_t{RollFiltered};
}

units::degree_t Pigeon::GetPitch()
{
  return units::degree_t{RobotPigeon.GetPitch()};
}

units::degree_t Pigeon::GetCorrectedRoll()
{
  return Pigeon::GetRoll() + RollOffset;
}

void Pigeon::RestPigeonAngle()
{
  RobotPigeon.SetFusedHeading(0.0);
}