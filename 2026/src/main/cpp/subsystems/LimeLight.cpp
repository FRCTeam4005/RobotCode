#include "subsystems/LimeLight.h"
#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include "subsystems/LimeLightHelpers/LimelightHelpers.h"
#include "Constants.h"

//TODO: Test that an overrun error is not occuring
LimeLight::LimeLight(LimeLightData* limelightdata, std::string tableName)
: LimeLightData_{limelightdata}
{
  table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);

  cameraServer = std::make_unique<std::thread>([&](){
    while(true){
      updateCameraData();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  });
}

  /**
   * TODO:
   * use table->GetNumberArray("botpose_wpired",std::vector<double>(6));// when on red side
  */
/**
 * defaults to postion based on that we are on the blue side at somepoint may pass in 
 * a string so that we know when we are on the red side.
 * Not sure when or where the based place to do this is at maybe the constructor.
*/
int LimeLight::updateCameraData() 
{  
  std::scoped_lock lk{networkTableMtx};
  //TODO: figure out if this still works with april tags?? 
  //* Only Updates information when there is a target avaliable

  if (!table->GetNumber("tv",0)) 
  {
    return -1;
  }

  LimeLightData_->Set_botPoseData(table.get());

  return 0;
}

void LimeLight::setLedOn() 
{
  std::scoped_lock lk{networkTableMtx};
  table->GetEntry("ledMode").SetInteger(LED_ON);
}

void LimeLight::setLedOff() 
{
  std::scoped_lock lk{networkTableMtx};
  table->GetEntry("ledMode").SetInteger(LED_OFF);
}

void LimeLight::setLedBlink() 
{
  std::scoped_lock lk{networkTableMtx};
  table->GetEntry("ledMode").SetInteger(LED_BLINK);
}