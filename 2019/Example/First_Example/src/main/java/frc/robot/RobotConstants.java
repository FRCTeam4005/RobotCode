package frc.robot;

public final class RobotConstants
{
    public static final int FR_DRIVE_TALONID = 18;
    public static final Boolean FR_DRIVE_INVERT = true;
    public static final int FR_TURN_TALONID = 17;
    public static final Boolean FR_TURN_INVERT = false;

    public static final int FL_DRIVE_TALONID = 20;
    public static final Boolean FL_DRIVE_INVERT= false; 
    public static final int FL_TURN_TALONID = 19;
    public static final Boolean FL_TURN_INVERT = false;

    public static final int BL_DRIVE_TALONID = 21;
    public static final Boolean BL_DRIVE_INVERT = false;
    public static final int BL_TURN_TALONID = 12;
    public static final Boolean BL_TURN_INVERT = false;

    public static final int BR_DRIVE_TALONID = 13;
    public static final Boolean BR_DRIVE_INVERT = true;
    public static final int BR_TURN_TALONID = 14;
    public static final Boolean BR_TURN_INVERT = false;

    public static final int ELEVATOR_MOTOR_LEFT_TALONID = 11;
    public static final Boolean ELEVATOR_MOTOR_LEFT_INVERT = true;
    public static final int ELEVATOR_MOTOR_RIGHT_TALONID = 22;
    public static final Boolean ELEVATOR_MOTOR_RIGHT_INVERT = false;

    public static final double ELEVATOR_TICKS_PER_REV = 256;

    public static final int ARM_TALONID = 15;
    public static final Boolean ARM_INVERT = false;
    public static final int INTAKE_TALONID = 16;
    public static final Boolean INTAKE_INVERT = false;

    public static final double DRIVEWHEELBASELENGTH = 18;
    public static final double DRIVETRACKWIDTH = 20;
    public static final double DRIVE_RADIUS = (Math.sqrt((DRIVEWHEELBASELENGTH * DRIVEWHEELBASELENGTH) + (DRIVETRACKWIDTH * DRIVETRACKWIDTH)));
    public static final double WHEEL_BASE_LENGTH_RADIUS_RATIO = (DRIVEWHEELBASELENGTH / DRIVE_RADIUS);
    public static final double TRACK_WIDTH_RADIUS_RATIO = (DRIVETRACKWIDTH / DRIVE_RADIUS);

    public static final double SWERVE_TICKS_PER_REV = 1657;


}