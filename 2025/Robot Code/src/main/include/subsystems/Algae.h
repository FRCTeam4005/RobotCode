#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/angle.h>
#include "Constants.h"
#include "RobotConstants.h"
#include <frc/DigitalInput.h>
#include <units/Angle.h>
#include <units/angular_velocity.h>
#include <units/Time.h>
#include <frc2/command/CommandPtr.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <frc2/command/SubsystemBase.h>

class Algae : public frc2::SubsystemBase {

    public:

        Algae();
        frc2::CommandPtr SetAlgaeCommand(units::turn_t goal);
        auto SetToCollect() -> frc2::CommandPtr;
        auto SetToScore() -> frc2::CommandPtr;
        auto GetOutput() -> double;
        auto SetToDesiredPosition(units::turn_t goal) -> frc2::CommandPtr;
        auto GetPosition() -> units::turn_t;
        auto SetRPM(double) -> frc2::CommandPtr;
        auto Outfeed() -> frc2::CommandPtr;
        auto Intake()-> frc2::CommandPtr;

        auto IsAlgaeArmInUpState() -> bool;


    private:

        std::unique_ptr<rev::spark::SparkMax> Algae_Motor_;
        std::unique_ptr<rev::spark::SparkBaseConfig> Algae_Motor_Config_;
        std::unique_ptr<rev::spark::SparkFlex> Intake_Motor_;
        std::unique_ptr<rev::spark::SparkBaseConfig> Intake_Motor_Config_;

        std::unique_ptr<rev::spark::SparkClosedLoopController> Algae_PID_;
        std::unique_ptr<rev::spark::SparkClosedLoopController> Intake_PID_;

        std::unique_ptr<rev::spark::SparkRelativeEncoder>  Algae_Encoder_;
        std::unique_ptr<rev::spark::SparkRelativeEncoder>  Intake_Encoder_;

        //Algae Arm Mag Sensor
        std::unique_ptr<frc::DigitalInput> AlgaeArm_UpSensor;
};