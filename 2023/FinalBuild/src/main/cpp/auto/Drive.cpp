#include "auto/Drive.h"

/**
 * @param fwd how far fwd the robot will travel down field, if negative then it will travel backwards. 
 * @param right how far right you want to go, If negative the robot will travel to the left.
*/
    RobotAutoDrive::RobotAutoDrive(units::meter_t fwd, units::meter_t right, units::radian_t theta, units::meters_per_second_t speed, Drivetrain &swervedrive)
    : Swervedrive{swervedrive}
    {
        fwd *= .75;
        right *= .75;
        //find the distance total that 
        auto DistanceT = (units::math::abs(fwd) + units::math::abs(right));

        //splits the speeds for the strafe and forward so that the robot
        //will move across the hypthoeneuse
        auto fwdSpeed = ratioSpeedCalc(fwd, DistanceT, speed);
        auto strSpeed = ratioSpeedCalc(right, DistanceT, speed);
        auto fwdAccel = ratioAccelCalc(fwd, DistanceT, Max_Accel);
        auto strAccel = ratioAccelCalc(right, DistanceT, Max_Accel);
        auto RCWAccel = 90_deg_per_s_sq; // these number were picked at random
        auto RCWSpeed = 90_deg_per_s; // these number were picked at random
     

        // go backwards if the fwd value is less than zero
        if(fwd.value() < 0)
        {
            fwdSpeed = -fwdSpeed;
            fwdAccel = -fwdAccel;
        }

        // go to the left if the right value is less than zero
        if(right.value() < 0)
        {
            strSpeed = -strSpeed;
            strAccel = -strAccel;
        }

        // the speed and acceleration that we want to go
        fwdConstraints = {fwdSpeed, fwdAccel};
        // the robots current speed and acceleration
        // we usually just stop and change direction so turns and things could be one continuous motion 
        fwdStartState = {0_m, 0_mps};
        // [fwd] the distance we would like to be by the end of the movement
        // [0_mps] the speed we would like to be going when this movement is over
        // we (for simplicity) just want to go a distance and stop so the end speed is 0_mps
        fwdEndState = {fwd, 0_mps};
        // just passing in the constraints into a profile which will be where we can call the calculate function
        fwdProfile = {fwdConstraints, fwdEndState, fwdStartState};

        // pretty much the same as the fwdConstraints and stuff
        strConstraints = {strSpeed, strAccel};
        strStartState = {0_m, 0_mps};
        strEndState = {right, 0_mps};
        strProfile = {strConstraints, strEndState, strStartState};  

        // pretty similar fwdConstraints and stuff
        RCWConstraints = {RCWSpeed, RCWAccel};
        RCWStartState = {0_deg, 0_deg_per_s};
        RCWEndState = {theta, 0_deg_per_s};
        RCWProfile = {RCWConstraints, RCWEndState, RCWStartState};     
    }

    void RobotAutoDrive::AutoInit()
    {
        autoTimer.Start();
    }

    int RobotAutoDrive::AutoRun()
    {
        if(AutoDone)
        {
           Swervedrive.autoSwerve(0_mps, 0_mps, 0_deg_per_s);
            return 0; 
        }

        auto fwdTrajectory = fwdProfile.Calculate(autoTimer.Get());
        auto strTrajectory = strProfile.Calculate(autoTimer.Get());
        auto RCWTrajectory = RCWProfile.Calculate(autoTimer.Get());

        if(autoTimer.AdvanceIfElapsed(fwdProfile.TotalTime()) && autoTimer.AdvanceIfElapsed(strProfile.TotalTime()) && autoTimer.AdvanceIfElapsed(RCWProfile.TotalTime()))
        {
            autoTimer.Stop();
            AutoDone = true;
        }

        Swervedrive.autoSwerve(fwdTrajectory.velocity, strTrajectory.velocity, RCWTrajectory.velocity);
        return -1;
    }