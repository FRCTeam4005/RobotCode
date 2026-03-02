#include "subsystems/LimeLightData.h"

LimeLightData::LimeLightData(std::optional<frc::DriverStation::Alliance> alliance)
{
  SetPoseSide(alliance.value());
}

void LimeLightData::Set_botPoseSide(std::optional<frc::DriverStation::Alliance> alliance)
{
  std::scoped_lock lk{botPoseMtx};
  SetPoseSide(alliance.value());
}

void LimeLightData::SetPoseSide(std::optional<frc::DriverStation::Alliance> side)
{
  //Get a value from the dashboard that we can set manually if the std::optional doesn't have a value
  if(!side.has_value())
  {
    //Get data from dashboard
    //auto dashboardSide = getdashboardside();
    //botPoseSide = "botpose_wpiblue";
    //if(dashboardSide == red)
    //{
    //  botPoseSide = "botpose_wpired";
    //}
  }
  else
  {
    botPoseSide = "botpose_wpiblue";
    if(side == frc::DriverStation::Alliance::kRed)
    {
      botPoseSide = "botpose_wpired";
    }
  }
}

std::string LimeLightData::Get_botPoseSide()
{
  std::scoped_lock lk{botPoseMtx};
  return botPoseSide;
}

void LimeLightData::Set_botPoseData(nt::NetworkTable* tbl)
{
  {
    std::scoped_lock lk{botPoseMtx};
    botPose = tbl->GetNumberArray(botPoseSide, std::vector<double>(6));
  }

  X.store(botPose[0]);
  Y.store(botPose[1]);
  Z.store(botPose[2]);
  Roll.store(botPose[3]);
  Pitch.store(botPose[4]);
  Yaw.store(botPose[5]);
  Tilda_ID.store(tbl->GetNumber("tid", 0));
  TargetArea.store(tbl->GetEntry("ta").GetFloat(0));

}
  
  
std::vector<double> LimeLightData::Get_botPoseVect()
{
  std::scoped_lock lk{botPoseMtx};
  return botPose;
}
