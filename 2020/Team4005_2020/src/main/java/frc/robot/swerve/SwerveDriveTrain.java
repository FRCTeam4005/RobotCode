package frc.robot.swerve;

import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PigeonData;
import frc.robot.subsystems.limelight.CameraAngles;
import frc.robot.subsystems.limelight.LimeLightVision;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotConstants;
import net.jafama.FastMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveTrain
{
  public static final SwerveDriveTrain instance = new SwerveDriveTrain();
  private static SwerveModule frModule;
  private static SwerveModule flModule;
  private static SwerveModule brModule;
  private static SwerveModule blModule;

  private static boolean fieldCentric = true;
  public static boolean targetAngleFromLimeLight = false;

  private static final double kPgain = 0.04; 				// percent throttle per degree of error */
	private static final double kDgain = 0.0004; 			// percent throttle per angular velocity dps */
  private static double _targetAngle = 0;

  private SwerveDriveTrain()
  {
    frModule = new SwerveModule( RobotConstants.FR_TURN_TALONID, RobotConstants.FR_TURN_INVERT, RobotConstants.FR_DRIVE_TALONID, RobotConstants.FR_DRIVE_INVERT);
    flModule = new SwerveModule( RobotConstants.FL_TURN_TALONID, RobotConstants.FL_TURN_INVERT, RobotConstants.FL_DRIVE_TALONID, RobotConstants.FL_DRIVE_INVERT);
    blModule = new SwerveModule( RobotConstants.BL_TURN_TALONID, RobotConstants.BL_TURN_INVERT, RobotConstants.BL_DRIVE_TALONID, RobotConstants.BL_DRIVE_INVERT);
    brModule = new SwerveModule( RobotConstants.BR_TURN_TALONID, RobotConstants.BR_TURN_INVERT, RobotConstants.BR_DRIVE_TALONID, RobotConstants.BR_DRIVE_INVERT);
  }

  private static boolean targetAngleUpdated = false;
  public void CalculateSwerve( double fwd, double str, double rcw )
  {
    fwd = Deadband(fwd);
    str = Deadband(str);
    rcw = Deadband(rcw);

    double fwd2 = 0;
    double str2 = 0;
    if(fieldCentric == true)
    {
      PigeonData pigeonData = Pigeon.instance.getPigeonData();
      CameraAngles cameraAngle = LimeLightVision.instance.getCameraAngles();
      if(rcw == 0)
      {
        if(targetAngleFromLimeLight == true && cameraAngle != null)
        {
          _targetAngle = pigeonData.fusionangle + cameraAngle.getTx();
        }
        else if(targetAngleUpdated == false)
        {
          _targetAngle = pigeonData.fusionangle;
          targetAngleUpdated = true;
        }

        rcw = ((_targetAngle - pigeonData.fusionangle) * kPgain) - (pigeonData.angularrate * kDgain);
        
        if(rcw < -1)
        {
          rcw = -1;
        }
        if(rcw > 1)
        {
          rcw = 1;
        }
      }
      else
      {
        targetAngleUpdated = false;
      }

      rcw *= 0.65;

      //ONLY FOR FIELD CENTRIC DRIVINGtgr
      double angle = -FastMath.toRadians(pigeonData.fusionangle);
      double temp = (fwd*FastMath.cos(angle) + str*FastMath.sin(angle));
      str2 = (-fwd*FastMath.sin(angle) + str*FastMath.cos(angle));
      fwd2 = temp;

      fwd2 = Deadband(fwd2);
      str2 = Deadband(str2);
    }
    else
    {
      fwd2 = fwd;
      str2 = str;
    }
    
    double a = str2 - (rcw * RobotConstants.WHEEL_BASE_LENGTH_RADIUS_RATIO);
    double b = str2 + (rcw * RobotConstants.WHEEL_BASE_LENGTH_RADIUS_RATIO);
    double c = fwd2 - (rcw * RobotConstants.TRACK_WIDTH_RADIUS_RATIO);
    double d = fwd2 + (rcw * RobotConstants.TRACK_WIDTH_RADIUS_RATIO);
    
    double frSpeed = FastMath.sqrt(FastMath.pow2(b) + FastMath.pow2(c));
    double flSpeed = FastMath.sqrt(FastMath.pow2(b) + FastMath.pow2(d));
    double blSpeed = FastMath.sqrt(FastMath.pow2(a) + FastMath.pow2(d));
    double brSpeed = FastMath.sqrt(FastMath.pow2(a) + FastMath.pow2(c));
    
    //Normalize wheel speeds as stated in Ether's document
    double max = frSpeed;
    if( flSpeed > max )
    {
      max = flSpeed;
    }
    if( blSpeed > max )
    {
      max = blSpeed;
    }
    if( brSpeed > max )
    {
      max = brSpeed;
    }
    if( max > 1 )
    {
      frSpeed/=max;
      flSpeed/=max;
      brSpeed/=max;
      blSpeed/=max;
    }

    double frAngle = FastMath.toDegrees(FastMath.atan2(b,c));
    double flAngle = FastMath.toDegrees(FastMath.atan2(b,d));
    double blAngle = FastMath.toDegrees(FastMath.atan2(a,d));
    double brAngle = FastMath.toDegrees(FastMath.atan2(a,c));

    double frEncoderPos = frModule.TurnMotor.getSelectedSensorPosition(0) / RobotConstants.SWERVE_TICKS_PER_REV;
    double flEncoderPos = flModule.TurnMotor.getSelectedSensorPosition(0) / RobotConstants.SWERVE_TICKS_PER_REV;
    double blEncoderPos = blModule.TurnMotor.getSelectedSensorPosition(0) / RobotConstants.SWERVE_TICKS_PER_REV;
    double brEncoderPos = brModule.TurnMotor.getSelectedSensorPosition(0) / RobotConstants.SWERVE_TICKS_PER_REV;

    double frPercent = frAngle / 360;
    double flPercent = flAngle / 360;
    double blPercent = blAngle / 360;
    double brPercent = brAngle / 360;

    double frConvertAngle = ConvertAngle( frPercent, frEncoderPos );
    double flConvertAngle = ConvertAngle( flPercent, flEncoderPos );
    double blConvertAngle = ConvertAngle( blPercent, blEncoderPos );
    double brConvertAngle = ConvertAngle( brPercent, brEncoderPos );

    if( ShouldReverse( frPercent, frEncoderPos) )
    {
      if( frConvertAngle < 0 )
      {
        frConvertAngle += 0.5;
      }
      else
      {
        frConvertAngle -= 0.5;
      }

      frSpeed = -frSpeed;
    }

    if( ShouldReverse(flPercent, flEncoderPos) )
    {
      if( flConvertAngle < 0 )
      {
        flConvertAngle += 0.5;
      }
      else
      {
        flConvertAngle -= 0.5;
      }

      flSpeed = -flSpeed;
    }

    if( ShouldReverse(blPercent, blEncoderPos) )
    {
      if( blConvertAngle < 0 )
      {
        blConvertAngle += 0.5;
      }
      else
      {
        blConvertAngle -= 0.5;
      }

      blSpeed = -blSpeed;
    }

    if( ShouldReverse(brPercent, brEncoderPos) )
    {
      if( brConvertAngle < 0 )
      {
        brConvertAngle += 0.5;
      }
      else
      {
        brConvertAngle -= 0.5;
      }

      brSpeed = -brSpeed;
    }

    double frTicks = frConvertAngle * RobotConstants.SWERVE_TICKS_PER_REV;
    double flTicks = flConvertAngle * RobotConstants.SWERVE_TICKS_PER_REV;
    double blTicks = blConvertAngle * RobotConstants.SWERVE_TICKS_PER_REV;
    double brTicks = brConvertAngle * RobotConstants.SWERVE_TICKS_PER_REV;

    if( frSpeed != 0 || flSpeed != 0 || blSpeed != 0 || brSpeed != 0 )
    {
      frModule.TurnMotor.set(ControlMode.Position, frTicks);
      flModule.TurnMotor.set(ControlMode.Position, flTicks);
      blModule.TurnMotor.set(ControlMode.Position, blTicks);
      brModule.TurnMotor.set(ControlMode.Position, brTicks);
    }
    
    //Wheel speeds are now 0-1. Not -1 to +1.
    //Multiply wheel speeds by 12 for voltage control mode
    frModule.DriveMotor.set(ControlMode.PercentOutput, frSpeed);
    flModule.DriveMotor.set(ControlMode.PercentOutput, flSpeed);
    blModule.DriveMotor.set(ControlMode.PercentOutput, blSpeed);
    brModule.DriveMotor.set(ControlMode.PercentOutput, brSpeed);

    SmartDashboard.putNumber("Fr Dist: ", frModule.DriveMotor.getSelectedSensorPosition()/427);
    SmartDashboard.putNumber("Fl Dist: ", flModule.DriveMotor.getSelectedSensorPosition()/427);
    SmartDashboard.putNumber("Bl Dist: ", blModule.DriveMotor.getSelectedSensorPosition()/427);
    SmartDashboard.putNumber("Br Dist: ", brModule.DriveMotor.getSelectedSensorPosition()/427);
  }

  boolean toggleRun = false;
  public void toggleFieldCentric(boolean toggle)
  {
    if(toggle == true && toggleRun == false)
    {
      if( fieldCentric == true )
      {
        fieldCentric = false;
      }
      else
      {
        fieldCentric = true;
      }
      toggleRun = true;
    }
    else if(!toggle)
    {
      toggleRun = false;
    }
  }

  /** 
     * @param value to cap.
   * @param peak positive double representing the maximum (peak) value.
   * @return a capped value.
   */
  double Cap(double value, double peak) 
  {
    if (value < -peak)
      return -peak;
    if (value > +peak)
      return +peak;
    return value;
  }
    
  /**
   * Given the robot forward throttle and ratio, return the max
   * corrective turning throttle to adjust for heading.  This is
   * a simple method of avoiding using different gains for
   * low speed, high speed, and no-speed (zero turns).
   */
  double MaxCorrection(double forwardThrot, double scalor) 
  {
    /* make it positive */
    if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
    /* max correction is the current forward throttle scaled down */
    forwardThrot *= scalor;
    /* ensure caller is allowed at least 10% throttle,
      * regardless of forward throttle */
    if(forwardThrot < 0.10)
      return 0.10;
    return forwardThrot;
  }

  public double getCMDistance()
  {
    double AvgDist = (frModule.DriveMotor.getSelectedSensorPosition() / 427);
    AvgDist += (flModule.DriveMotor.getSelectedSensorPosition() / 427);
    AvgDist += (blModule.DriveMotor.getSelectedSensorPosition() / 427);
    AvgDist += (brModule.DriveMotor.getSelectedSensorPosition() / 427);
    return Math.abs(AvgDist / 4);
  }

  public void zeroDistanceReading()
  {
    frModule.DriveMotor.setSelectedSensorPosition(0);
    flModule.DriveMotor.setSelectedSensorPosition(0);
    blModule.DriveMotor.setSelectedSensorPosition(0);
    brModule.DriveMotor.setSelectedSensorPosition(0);
  }

  public void TankDrive( double left, double right )
  {
    flModule.TurnMotor.set(ControlMode.Position, 0);
    flModule.DriveMotor.set(ControlMode.PercentOutput, left);

    frModule.TurnMotor.set(ControlMode.Position, 0);
    frModule.DriveMotor.set(ControlMode.PercentOutput, left);

    blModule.TurnMotor.set(ControlMode.Position, 0);
    blModule.DriveMotor.set(ControlMode.PercentOutput, right);

    brModule.TurnMotor.set(ControlMode.Position, 0);
    brModule.DriveMotor.set(ControlMode.PercentOutput, right);
  }

  double Deadband(double value)
  {
    if(value < -0.05)
    {
      return value;
    }

    if(value > 0.05)
    {
      return value;
    }
    return 0.0;
  }

  boolean ShouldReverse( double percentLocation, double encoderValue )
  {
      long WholeNum = 0;
      WholeNum = (long)encoderValue;
      double fractionOfEncoder = encoderValue - WholeNum; //fractional part of the postion

      if( percentLocation < 0 )
      {
          percentLocation += 1;
      }

      double longDiff = FastMath.abs(percentLocation - fractionOfEncoder);
      double diff = FastMath.min( longDiff, 1.0 - longDiff );

      if( diff > 0.25 )
      {
          return true;
      }

      return false;
  } 

  double ConvertAngle( double percentLocation, double encoderValue )
  {
    double temp = percentLocation;
    temp += (int)encoderValue;

    long WholeNum = 0;
    WholeNum = (long)encoderValue;
    double fractionOfEncoder = encoderValue - WholeNum; //fractional part of the postion

    if( (percentLocation - fractionOfEncoder) > 0.5 )
    {
        temp -= 1;
    }
    if( (percentLocation - fractionOfEncoder) < -0.5 )
    {
        temp += 1;
    }

    return temp;
  }
}
