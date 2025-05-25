package frc.robot;

import frc.robot.SwerveModule;
import com.ctre.phoenix.motorcontrol.ControlMode;
import static frc.robot.RobotConstants.*;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SPI;
import net.jafama.FastMath;

public class SwerveDriveTrain
{
    private static SwerveDriveTrain instance;
    private static SwerveModule frModule;
    private static SwerveModule flModule;
    private static SwerveModule brModule;
    private static SwerveModule blModule;
    //private static final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    public static SwerveDriveTrain getInstance()
    {
        if( instance == null )
        {
            instance = new SwerveDriveTrain();
        }

        return instance;
    }

    private SwerveDriveTrain(  )
    {
        frModule = new SwerveModule( FR_TURN_TALONID, FR_TURN_INVERT, FR_DRIVE_TALONID, FR_DRIVE_INVERT);
        flModule = new SwerveModule( FL_TURN_TALONID, FL_TURN_INVERT, FL_DRIVE_TALONID, FL_DRIVE_INVERT);
        blModule = new SwerveModule( BL_TURN_TALONID, BL_TURN_INVERT, BL_DRIVE_TALONID, BL_DRIVE_INVERT);
        brModule = new SwerveModule( BR_TURN_TALONID, BR_TURN_INVERT, BR_DRIVE_TALONID, BR_DRIVE_INVERT);
    }
    
    public void ZeroSwerve ()
    {
        frModule.TurnMotor.set(ControlMode.Position, 0);
        flModule.TurnMotor.set(ControlMode.Position, 0);
        blModule.TurnMotor.set(ControlMode.Position, 0);
        brModule.TurnMotor.set(ControlMode.Position, 0);
    }

    public void CalculateSwerve( double fwd, double str, double rcw, long elevatorPos  )
    {   
        if( FastMath.abs(fwd) < 0.1 )
        {
            fwd = 0;
        }
        if( FastMath.abs(str) < 0.1 )
        {
            str = 0;
        }
        if( FastMath.abs(rcw) < 0.1 )
        {
            rcw = 0;
        }

        rcw *= 0.75;
        
        //ONLY FOR FIELD CENTRIC DRIVING
        //TODO: Figure this out later
        //double angle = FastMath.toRadians(gyro.getAngle());
        //double temp = (fwd*FastMath.cos(angle) + str*FastMath.sin(angle));
        //double str2 = (-fwd*FastMath.sin(angle) + str*FastMath.cos(angle));
        //double fwd2 = temp;
        
        double a = str - (rcw * WHEEL_BASE_LENGTH_RADIUS_RATIO);
        double b = str + (rcw * WHEEL_BASE_LENGTH_RADIUS_RATIO);
        double c = fwd - (rcw * TRACK_WIDTH_RADIUS_RATIO);
        double d = fwd + (rcw * TRACK_WIDTH_RADIUS_RATIO);
        
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

        double frEncoderPos = frModule.TurnMotor.getSelectedSensorPosition(0) / SWERVE_TICKS_PER_REV;
        double flEncoderPos = flModule.TurnMotor.getSelectedSensorPosition(0) / SWERVE_TICKS_PER_REV;
        double blEncoderPos = blModule.TurnMotor.getSelectedSensorPosition(0) / SWERVE_TICKS_PER_REV;
        double brEncoderPos = brModule.TurnMotor.getSelectedSensorPosition(0) / SWERVE_TICKS_PER_REV;

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

        double frTicks = frConvertAngle * SWERVE_TICKS_PER_REV;
        double flTicks = flConvertAngle * SWERVE_TICKS_PER_REV;
        double blTicks = blConvertAngle * SWERVE_TICKS_PER_REV;
        double brTicks = brConvertAngle * SWERVE_TICKS_PER_REV;

        if( frSpeed != 0 || flSpeed != 0 || blSpeed != 0 || brSpeed != 0 )
        {
            frModule.TurnMotor.set(ControlMode.Position, frTicks);
            flModule.TurnMotor.set(ControlMode.Position, flTicks);
            blModule.TurnMotor.set(ControlMode.Position, blTicks);
            brModule.TurnMotor.set(ControlMode.Position, brTicks);
        }

        double Scaling = 0.85;
        long elevatorHalfHeight = (70830 / 2);
        if( elevatorPos > elevatorHalfHeight )
        {
            /*long scaledElevtorPos = (elevatorPos / 2);
            double adjustment = (double)((elevatorHalfHeight - scaledElevtorPos) / elevatorHalfHeight);
            if( adjustment > 0.6 )
            {
                adjustment = 0.6;
            }*/
            
            double adjustment = (0.6/elevatorHalfHeight)*(elevatorPos-elevatorHalfHeight);

            Scaling = Scaling - adjustment;
        }
        
        //Wheel speeds are now 0-1. Not -1 to +1.
        //Multiply wheel speeds by 12 for voltage control mode
        frModule.DriveMotor.set(ControlMode.PercentOutput, frSpeed * Scaling);
        flModule.DriveMotor.set(ControlMode.PercentOutput, flSpeed * Scaling);
        blModule.DriveMotor.set(ControlMode.PercentOutput, blSpeed * Scaling);
        brModule.DriveMotor.set(ControlMode.PercentOutput, brSpeed * Scaling);
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

    private static boolean ShouldReverse( double percentLocation, double encoderValue )
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

    private static double ConvertAngle( double percentLocation, double encoderValue )
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