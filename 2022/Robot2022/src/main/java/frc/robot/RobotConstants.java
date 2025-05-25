package frc.robot;

public class RobotConstants {
    
public static final int elevator_control = 56;
public static final int turret_control = 20;

public static final int left_falcon = 57;
public static final int right_falcon = 58;

public static final int Intake_spin = 60;
public static final int Intake_out = 59;

public static final int PIGEON_TALON_ID = 55;

public static final int ELEVATOR_MOTOR = 22;
public static final int OTHER_ELEVATOR_MOTOR = 21;

//SWERVE MODULES

//Front Left Swerve Module
public static final int FL_DRIVE_ID = 61 ;
public static final boolean FL_DRIVE_INVERT = true;
public static final double FL_DRIVE_KP = 6e-5;

public static final int FL_TURN_ID = 60;
public static final boolean FL_TURN_INVERT = false;
public static final int FL_CANCODER = 52;

//Front Right Swerve Module
public static final int FR_DRIVE_ID = 55 ;
public static final boolean FR_DRIVE_INVERT = true;
public static final double FR_DRIVE_KP = 6e-5;

public static final int FR_TURN_ID = 54;
public static final boolean FR_TURN_INVERT = false;
public static final int FR_CANCODER = 51;
public static final double FR_cancoder_offset = 0;

//Back Left Swerve Module
public static final int BL_DRIVE_ID = 59 ;
public static final boolean BL_DRIVE_INVERT = true;
public static final double BL_DRIVE_KP = 6e-5;

public static final int BL_TURN_ID = 58;
public static final boolean BL_TURN_INVERT = false;
public static final int BL_CANCODER = 53;

//Back Right Swerve Module
public static final int BR_DRIVE_ID = 57 ;
public static final boolean BR_DRIVE_INVERT = true;

public static final double BR_DRIVE_KP = 6e-5;

public static final int BR_TURN_ID = 56;
public static final boolean BR_TURN_INVERT = false;
public static final int BR_CANCODER = 54;

//END OF SWERVE MODULES

// adjustable things

//MAX SPEEDS
//max speed must be less than or equal to 4
public static final double MAX_ANGULAR_SPEED = 2;
public static final double MAX_FORWARD_SPEED = 2;
public static final double MAX_STRAFE_SPEED = 2;

public static final double elevator_speed = 0.80;

public static final double F_LINEAR_AC = 0;
public static final double P_LINEAR_AC = 0.55;
public static final double I_LINEAR_AC = 0.0001;
public static final double D_LINEAR_AC = 0.001;
public static final int INT_ZONE_LINEAR_AC = 500;
public static final double PEAK_OUT_LINEAR_AC = 0.85;
public static final int ACCEL_LINEAR_AC = 500;
public static final int CRUISE_LINEAR_AC = 1500;



}
