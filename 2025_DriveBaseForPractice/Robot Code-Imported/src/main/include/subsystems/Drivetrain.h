// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "subsystems/SwerveModule.h"
#include <DriverController.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <wpi/array.h>
#include <frc/smartdashboard/Field2d.h>


using namespace SwerveDriveConstants;


class Drivetrain : public frc2::SubsystemBase
{
public:
  Drivetrain(ctre::phoenix6::hardware::Pigeon2&);  

  void SetChassisSpeeds( const frc::ChassisSpeeds& speeds,
              units::second_t period = SwerveDriveConstants::kDrivePeriod );

  void StopAllMotors_();
  auto GetisDriving() -> bool { return isDriving_; }

  frc::Pose2d GetPose();
  void ResetPose(const frc::Pose2d& pose);
  auto SetYaw(units::degree_t angle) -> void;
  auto ResetOdoWithCamera() -> void;
  auto UpdateOdoWithCamera() -> void;
  auto getRotation2D() -> frc::Rotation2d;
  void Periodic() override;
  void AutoDrive_( const frc::ChassisSpeeds& speeds);




 private:
  std::unique_ptr<SwerveModule> frModule_;
  std::unique_ptr<SwerveModule> flModule_;
  std::unique_ptr<SwerveModule> brModule_;
  std::unique_ptr<SwerveModule> blModule_;

  std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> PoseEstimator;

  ctre::phoenix6::hardware::Pigeon2& pigeon_;

  bool isDriving_ = false;

  // void InitSendable(wpi::SendableBuilder& builder) override;
  void SetAllModuleStates_(const wpi::array<frc::SwerveModuleState, 4u> &moduleStates);
  auto GetAllSwerveModulePositions_() -> wpi::array<frc::SwerveModulePosition, 4>;
  auto GetCameraPose() -> frc::Pose2d;
  
  
  inline frc::ChassisSpeeds getSpeeds_() 
  {
    return SwerveDriveConstants::Kinematics.ToChassisSpeeds({flModule_->getState() , frModule_->getState(), blModule_->getState(), brModule_->getState()});
  }  
};

