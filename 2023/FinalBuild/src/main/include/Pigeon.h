
#ifndef PIGEON_H
#define PIGEON_H

#include <memory>
#include "RobotConstants.h"
#include <ctre/Phoenix.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <units/angle.h>

#define PIGEON2

struct PigeonData
{
  PigeonData(units::degree_t fusionAngle, double angularRate, bool goodAngle)
  : FusionAngle{fusionAngle}, AngularRate{angularRate}, GoodAngle{goodAngle}
  {}

  units::degree_t FusionAngle;
  double AngularRate;
  bool GoodAngle;
};


class Pigeon
{
public:
  Pigeon(Pigeon &other) = delete;
  Pigeon(Pigeon &&other) = delete;
  void operator=(const Pigeon &) = delete;
  void operator=(const Pigeon &&) = delete;

  static Pigeon& GetInstance();

  PigeonData GetPigeonData();
  void RestPigeonAngle();

  units::degrees_per_second_t GetYawRate();

  units::degree_t GetRoll();
  units::degree_t GetPitch();
  units::degree_t GetCorrectedRoll();

private:

  WPI_PigeonIMU RobotPigeon{PIGEON_ID};



  Pigeon();
  units::degree_t RollOffset;
  double RollFiltered;
};

#endif