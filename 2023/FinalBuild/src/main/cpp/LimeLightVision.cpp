#include "LimeLightVision.h"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/dimensionless.h>

static const LimeLightVision instance = LimeLightVision();

/**
 * Utility class to calculate distances based off of LimeLIght camera angle calculation.
 * @param cameraHeight height of Limelight camera, in inches
 * @param targetHeight height of field target, in inches
 * @param cameraAngle camera mounting angle, in degrees, where 0 is horizontal and down is negative
 */
LimeLightVision::LimeLightVision()
{}



/**
 * Calculate and return the distances from the camera.
 * The calculated distances are in inches, as calculated by the triangulation of the vertical distance between the
 * camera and target and the detected angles.
 * forward is the "straight" distance from the camera to the target
 * sideways is the "lateral" distance from the camera to the target
 * @return Camera distance calculated from the limelight, null if no target acquired
 */
int LimeLightVision::getTargetDistance(CameraDistance& cameradistance ) 
{
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  tv = table->GetBoolean("tv",0.0);
  tildaID = table->GetNumber("tid", 0);

  // frc::SmartDashboard::PutNumber("tx = table->GetNumber(tx,0.0);", tx);
  // frc::SmartDashboard::PutNumber("ty = table->GetNumber(ty,0.0);", ty);
  // frc::SmartDashboard::PutNumber("ta = table->GetNumber(ta,0.0);", ta);
  // frc::SmartDashboard::PutNumber("tv = table->GetNumber(tv,0.0);", tv);
  // frc::SmartDashboard::PutNumber("tildaID = table->GetNumber(tid, 0);", tildaID);


  if (!tv) 
  {
    return -1;
  }


  cameradistance.forward = (cameraHeight - targetHeight) / tan(cameraAngle.value() + -ty);
  cameradistance.sideways = cameradistance.forward * std::tan(units::radian_t(tx).value());
  cameradistance.Tilda_ID = tildaID;

  frc::SmartDashboard::PutNumber("cameradistance.forward ", cameradistance.forward.value());
  frc::SmartDashboard::PutNumber("cameradistance.sideways", cameradistance.sideways.value());

  return 0;
}

bool LimeLightVision::if_valid_target()
{
  return (table->GetNumber("tv",0.0) > .001);
}

void LimeLightVision::setLedOn() 
{
  table->GetEntry("ledMode").SetInteger(LED_ON);
}

void LimeLightVision::setLedOff() 
{
  table->GetEntry("ledMode").SetInteger(LED_OFF);
}

void LimeLightVision::setLedBlink() 
{
  table->GetEntry("ledMode").SetInteger(LED_BLINK);
}

