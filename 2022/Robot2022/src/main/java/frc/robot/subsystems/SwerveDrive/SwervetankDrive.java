package frc.robot.subsystems.SwerveDrive;

import frc.robot.RobotConstants;
import frc.robot.subsystems.pigeon.Pigeon;
import frc.robot.subsystems.pigeon.PigeonData;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwervetankDrive
{
  private static Translation2d m_frontLeftLocation = new Translation2d(0.305, 0.2143);
  private static Translation2d m_frontRightLocation = new Translation2d(0.305, -0.2143);
  private static Translation2d m_backLeftLocation = new Translation2d(-0.305, 0.2143);
  private static Translation2d m_backRightLocation = new Translation2d(-0.305, -0.2143);

  private static SwerveDriveKinematics m_Kinematics;

  public static  final SwervetankDrive instance = new SwervetankDrive();
  public static SwerveModule frModule;
  public static SwerveModule flModule;
  public static SwerveModule brModule;
  public static SwerveModule blModule;

  public SwerveModuleState flState;
  public SwerveModuleState frState;
  public SwerveModuleState blState;
  public SwerveModuleState brState;

  private static CANCoder flCanCoder;
  private static CANCoder frCanCoder;
  private static CANCoder blCanCoder;
  private static CANCoder brCanCoder;
  
  private SwervetankDrive()
  {
    flCanCoder = new CANCoder(RobotConstants.FL_CANCODER);
    frCanCoder = new CANCoder(RobotConstants.FR_CANCODER);
    blCanCoder = new CANCoder(RobotConstants.BL_CANCODER);
    brCanCoder = new CANCoder(RobotConstants.BR_CANCODER);
    flCanCoder.setPositionToAbsolute();
    frCanCoder.setPositionToAbsolute();
    blCanCoder.setPositionToAbsolute();
    brCanCoder.setPositionToAbsolute();

    frCanCoder.configSensorDirection(true);
    flCanCoder.configSensorDirection(true);
    //blCanCoder.configSensorDirection(true);
   // brCanCoder.configSensorDirection(true);

  


    m_Kinematics = new SwerveDriveKinematics
    (
      m_frontRightLocation, m_frontLeftLocation, 
      m_backRightLocation, m_backLeftLocation
    );

    flModule = new SwerveModule
    (RobotConstants.FL_DRIVE_ID,RobotConstants.FL_DRIVE_INVERT, RobotConstants.FL_DRIVE_KP, RobotConstants.FL_TURN_ID, RobotConstants.FL_TURN_INVERT);

    frModule = new SwerveModule
    (RobotConstants.FR_DRIVE_ID,RobotConstants.FR_DRIVE_INVERT, RobotConstants.FR_DRIVE_KP, RobotConstants.FR_TURN_ID, RobotConstants.FR_TURN_INVERT);

  
    blModule = new SwerveModule
    (RobotConstants.BL_DRIVE_ID,RobotConstants.BL_DRIVE_INVERT, RobotConstants.BL_DRIVE_KP, RobotConstants.BL_TURN_ID, RobotConstants.BL_TURN_INVERT);

    brModule = new SwerveModule
    (RobotConstants.BR_DRIVE_ID,RobotConstants.BR_DRIVE_INVERT, RobotConstants.BR_DRIVE_KP, RobotConstants.BR_TURN_ID, RobotConstants.BR_TURN_INVERT);
  }

  public void calculateSwerve(double fwd, double str, double rot)
  {
    fwd = DeadBand(fwd) * RobotConstants.MAX_FORWARD_SPEED;
    str = DeadBand(str) * RobotConstants.MAX_STRAFE_SPEED;
    rot = DeadBand(rot) * RobotConstants.MAX_ANGULAR_SPEED;

    double fwd2 = 0;
    double str2 = 0;

    PigeonData pigeonData = Pigeon.instance.getPigeonData();
    //ONLY FOR FIELD CENTRIC DRIVINGtgr
    double angle = -Math.toRadians(pigeonData.fusionangle);
    double temp = (fwd*Math.cos(angle) + str*Math.sin(angle));
    str2 = (-fwd*Math.sin(angle) + str*Math.cos(angle));
    fwd2 = temp;

    ChassisSpeeds speeds = new ChassisSpeeds(fwd2, str2, rot);
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
  
    flState = moduleStates[0];
    frState = moduleStates[1];
    blState = moduleStates[2];
    brState = moduleStates[3];

    if (speeds.omegaRadiansPerSecond != 0 || speeds.vxMetersPerSecond != 0 || speeds.vyMetersPerSecond != 0)
    {
    setDesiredState(flState, flCanCoder.getPosition(), flModule);
    setDesiredState(frState, frCanCoder.getPosition(), frModule);
    setDesiredState(blState, blCanCoder.getPosition(), blModule);
    setDesiredState(brState, brCanCoder.getPosition(), brModule);
    }
    else
    {
      flModule.Turn_Motor.set(0);
      flModule.Drive_Motor.set(0);
      frModule.Turn_Motor.set(0);
      frModule.Drive_Motor.set(0);
      blModule.Turn_Motor.set(0);
      blModule.Drive_Motor.set(0);
      brModule.Turn_Motor.set(0);
      brModule.Drive_Motor.set(0);
    }

    //SmartDashboard.putNumber("x speed", speeds.vxMetersPerSecond);
    //SmartDashboard.putNumber("y speeds", speeds.vyMetersPerSecond);
    //SmartDashboard.putNumber("angular", speeds.omegaRadiansPerSecond);
  }

  
  public void realign()
  {
    flModule.Turn_Motor.set(flModule.Turn_pid.calculate(flCanCoder.getPosition(), 0));
    frModule.Turn_Motor.set(frModule.Turn_pid.calculate(frCanCoder.getPosition(), 0));
    blModule.Turn_Motor.set(blModule.Turn_pid.calculate(blCanCoder.getPosition(), 0));
    brModule.Turn_Motor.set(brModule.Turn_pid.calculate(brCanCoder.getPosition(), 0));
  }

  public double DeadBand(double value)
  {
    if (Math.abs(value) > 0.08)
    {
      return value;
    }
    else
    {
      return 0;
    }
  }

  private void setDesiredState(SwerveModuleState DesiredState, double angle, SwerveModule module)
  {
    SwerveModuleState state = SwerveModuleState.optimize(DesiredState, new Rotation2d(Math.toRadians(angle)));
    module.Turn_Motor.set(module.Turn_pid.calculate(angle, state.angle.getDegrees()));
    module.Drive_Motor.set(state.speedMetersPerSecond * .55);
    //module.drive_Speed(DesiredState.speedMetersPerSecond);
  }
}