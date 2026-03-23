// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "subsystems/Shooter/Wheels.h"
#include <functional>

class ShooterWheels : public frc2::SubsystemBase 
{
 public:
  ShooterWheels(std::function<frc::Pose2d()> getBotPose);
  auto Toggle() -> frc2::CommandPtr;
  auto Spin() -> frc2::CommandPtr;
  auto Stop() -> frc2::CommandPtr;
  auto shootToDistance(std::function<void()> getDistance) -> frc2::CommandPtr;
  
private:
  units::turns_per_second_t ShootSpeed_;

  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> LeftMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> RightMotor;
  ctre::phoenix6::configs::Slot0Configs pid;

  void Periodic() override;
  void setSpeed(units::turns_per_second_t speed);
  void setNeutral();
  std::function<frc::Pose2d()> _getPose;

  #define BLUE_LINE_COORD 4.625594_m
  #define RED_LINE_COORD  12.563094_m
  #define MID_FIELD_LINE 4.0211375_m


  const frc::Translation2d SauronBlue = frc::Translation2d(BLUE_LINE_COORD, 4.034663_m);
  const frc::Translation2d LeftPassBlue = frc::Translation2d(1_m, 1.5_m);
  const frc::Translation2d RightPassBlue = frc::Translation2d(1_m, 7_m);
  
  const frc::Translation2d SauronRed = frc::Translation2d(11.915394_m, 4.034663_m);
  const frc::Translation2d LeftPassRed = frc::Translation2d(15.5_m, 1.5_m);
  const frc::Translation2d RightPassRed = frc::Translation2d(15.5_m, 7_m);


  frc::Translation2d getTargetTranlation(frc::Pose2d RobotPose)
  {

  // frc::Translation2d DesiredAimCoords;

  // if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed)
  // {
  //   if (RobotPose.X() > RED_LINE_COORD)
  //   {
  //     DesiredAimCoords = SauronRed;
  //   }
  //   else
  //   {

  //     if(RobotPose.Y() < MID_FIELD_LINE)
  //     {
  //       DesiredAimCoords = (LeftPassRed);
  //     }
  //     else
  //     {
  //       DesiredAimCoords = (RightPassRed);
  //     }
  //   }
  // }
  // else
  // {
  //   if (RobotPose.X() < BLUE_LINE_COORD)
  //   {
  //     DesiredAimCoords = (SauronBlue);
  //   }
  //   else
  //   {
  //     if(RobotPose.Y() < MID_FIELD_LINE)
  //     {
  //       DesiredAimCoords = (LeftPassBlue);
  //     }
  //     else
  //     {
  //       DesiredAimCoords = (RightPassBlue);
  //     }
  //   }
  // }

  // return DesiredAimCoords;
  return SauronRed;
}

units::meter_t getTargetDistance(frc::Translation2d TargetPose, frc::Pose2d RobotPose)
{
  frc::Translation2d DeltaPose{TargetPose.X() - RobotPose.X(), TargetPose.Y() - RobotPose.Y()};

  return units::meter_t{std::sqrt(DeltaPose.Y().value() * DeltaPose.Y().value() + DeltaPose.X().value() * DeltaPose.X().value())};
}
};
