package frc.robot;

public final class RobotConstants
{
    //FRONT RIGHT SWERVE
    public static final int FR_DRIVE_TALONID = 16;
    public static final Boolean FR_DRIVE_INVERT = true;
    public static final int FR_TURN_TALONID = 3;
    public static final Boolean FR_TURN_INVERT = false;

    //BACK RIGHT SWERVE
    public static final int BR_DRIVE_TALONID = 18;
    public static final Boolean BR_DRIVE_INVERT = true;
    public static final int BR_TURN_TALONID = 4;
    public static final Boolean BR_TURN_INVERT = false;

    //FRONT LEFT SWERVE
    public static final int FL_DRIVE_TALONID = 15;
    public static final Boolean FL_DRIVE_INVERT= false; 
    public static final int FL_TURN_TALONID = 2;
    public static final Boolean FL_TURN_INVERT = false;

    //BACK LEFT SWERVE
    public static final int BL_DRIVE_TALONID = 17;
    public static final Boolean BL_DRIVE_INVERT = false;
    public static final int BL_TURN_TALONID = 1;
    public static final Boolean BL_TURN_INVERT = false;

    public static final int SHOOTER_TALONID = 5;
    public static final int PIGEON_TALONID = 10;

    public static final int CONVEYOR_TALONID = 9;
    public static final int INTAKE_TALONID = 8;

    public static final int COMPRESSOR_PCM_CANID = 21;

    public static final int SOLENOID_PCM_CANID = 22;
    public static final int SOLENOID_LIFTID_OUT = 0;
    public static final int SOLENOID_LIFTID_IN = 1;
    public static final int SOLENOID_FEEDERID_OUT = 2;
    public static final int SOLENOID_FEEDERID_IN = 3;

    public static final int PORT_LINEAR_AC = 19;
    public static final int SB_LINEAR_AC = 20;

    public static final double UP_POSITION_LINEAR_AC = 300000;
    public static final double DOWN_POSITION_LINEAR_AC = 100;
    public static final double MIDDLE_POSITION_LINEAR_AC = 120000;
    
    public static final double F_LINEAR_AC = 0;
    public static final double P_LINEAR_AC = 0.75;
    public static final double I_LINEAR_AC = 0.00001;
    public static final double D_LINEAR_AC = 0;
    public static final int INT_ZONE_LINEAR_AC = 500;
    public static final double PEAK_OUT_LINEAR_AC = 1;
    public static final int ACCEL_LINEAR_AC = 500;
    public static final int CRUISE_LINEAR_AC = 1000;

    public static final double DRIVEWHEELBASELENGTH = 24;
    public static final double DRIVETRACKWIDTH = 16.875;
    public static final double DRIVE_RADIUS = (Math.sqrt((DRIVEWHEELBASELENGTH * DRIVEWHEELBASELENGTH) + (DRIVETRACKWIDTH * DRIVETRACKWIDTH)));
    public static final double WHEEL_BASE_LENGTH_RADIUS_RATIO = (DRIVEWHEELBASELENGTH / DRIVE_RADIUS);
    public static final double TRACK_WIDTH_RADIUS_RATIO = (DRIVETRACKWIDTH / DRIVE_RADIUS);

    public static final double SWERVE_TICKS_PER_REV = 1657;
}