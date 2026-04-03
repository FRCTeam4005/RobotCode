// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/HootAutoReplay.hpp"

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <optional>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;





private:
    static constexpr bool kUseLimelight = false;

    std::optional<frc2::CommandPtr> m_autonomousCommand;
    frc::SendableChooser<std::string> m_chooser;

    std::vector<std::string> AvaliablePathPlannerAutos =
    {
        "Sit and Shoot",
        "Depot Left",
        "Left Wall Auto",
        "Middle to Depot"
    };

    auto SetRobotAutoRoutine(std::string AutomousRoutineName) -> void {m_container.SetAutonomousRoutine(AutomousRoutineName);};
    auto GetRobotAutoCommand() -> frc2::CommandPtr {return m_container.GetAutonomousCommand();};

    RobotContainer m_container;

    /* log and replay timestamp and joystick data */
    ctre::phoenix6::HootAutoReplay m_timeAndJoystickReplay = ctre::phoenix6::HootAutoReplay{}
        .WithTimestampReplay()
        .WithJoystickReplay();
};
