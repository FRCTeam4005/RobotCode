package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import net.jafama.FastMath;

import static frc.robot.RobotConstants.*;

public class Elevator
{
    private static Elevator instance;
    //Right and left standing behind the robot
    private final TalonSRX ElevatorLeft;
    private final TalonSRX ElevatorRight;
    private final TalonSRX arm;
    private final TalonSRX intake;

    private double currentLeft = 0;
    private double currentRight = 0;
    private double CurrentPos = 0;
    private double CurrentArmPos = 0;
    private double rumblevar = 0;

    public static Elevator getInstance()
    {
        if( instance == null )
        {
            instance = new Elevator();
        }

        return  instance;
    }

    private Elevator()
    {
        ElevatorLeft = new TalonSRX(ELEVATOR_MOTOR_LEFT_TALONID);
        ElevatorRight = new TalonSRX(ELEVATOR_MOTOR_RIGHT_TALONID);

        arm = new TalonSRX(ARM_TALONID);
        arm.setNeutralMode(NeutralMode.Coast);
        intake = new TalonSRX(INTAKE_TALONID);
    }

    public void ElevatorInit()
    {
        
        final double f_ = 0;
        final double p_ = 1;
        final double i_ = 0.00001;
        final double d_ = 0;

        ElevatorLeft.configFactoryDefault();
        ElevatorLeft.setNeutralMode(NeutralMode.Brake);
        ElevatorLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        ElevatorLeft.setSelectedSensorPosition(0);
        ElevatorLeft.selectProfileSlot(0,0);
        ElevatorLeft.config_kF(0, f_, 50);
        ElevatorLeft.config_kP(0, p_, 50);
        ElevatorLeft.config_kI(0, i_, 50);
        ElevatorLeft.config_kD(0, d_, 50);
        ElevatorLeft.setInverted(ELEVATOR_MOTOR_LEFT_INVERT);

        ElevatorRight.configFactoryDefault();
        ElevatorRight.setNeutralMode(NeutralMode.Brake);
        ElevatorRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        ElevatorRight.setSelectedSensorPosition(0);
        ElevatorRight.selectProfileSlot(0,0);
        ElevatorRight.config_kF(0, f_, 50);
        ElevatorRight.config_kP(0, p_, 50);
        ElevatorRight.config_kI(0, i_, 50);
        ElevatorRight.config_kD(0, d_, 50);
        ElevatorRight.setInverted(ELEVATOR_MOTOR_RIGHT_INVERT);

        currentLeft = 0;
        currentRight = 0;
        CurrentPos = 0;
        CurrentArmPos = -590;

        arm.configFactoryDefault();
        arm.setNeutralMode(NeutralMode.Brake);
        arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        arm.setSelectedSensorPosition(-590);
        arm.selectProfileSlot(0,0);
        arm.config_kF(0, 0, 50);
        arm.config_kP(0, 25, 50);
        arm.config_kI(0, 0.00001, 50);
        arm.config_kD(0, 0.8, 50);



        intake.configFactoryDefault();
        intake.setNeutralMode(NeutralMode.Brake);
    }

    //right standing at the back of the robot
    public void TuneLiftRopesRIGHT(final XboxController xB)
    {
        if(xB.getRawButton(6))
        {
            currentRight -= 250;
        }

        if( xB.getRawButton(8) )
        {
            currentRight += 250;
        }
    }

    public void TuneLiftRopesLEFT(final XboxController xB)
    {
        //Back button
        if( xB.getRawButton(5) )
        {
            currentLeft -= 250;
        }

        //Start button
        if( xB.getRawButton(7) )
        {
            currentLeft += 250;
        }
    }

    public void TestArmLift(final XboxController xB)
    {
        final double ax = -xB.getRawAxis(1);

        final double ratearm = 3;
        if(ax <= -0.25 || ax >= 0.25){
            CurrentArmPos += -ax*ratearm;
        }
        if(CurrentArmPos < -590){CurrentArmPos = -590;}
        else if(CurrentArmPos > 50){CurrentArmPos = 50;}
        arm.set(ControlMode.Position, CurrentArmPos);

    }

    public void TestIntake(final XboxController xB)
    {
        if(xB.getRawButton(1))
        {
            intake.set(ControlMode.PercentOutput, -1);
        }
        else if(xB.getRawButton(2))
        {
            intake.set(ControlMode.PercentOutput, 1);
        }
        else
        {
            intake.set(ControlMode.PercentOutput, 0);
        }
    }

    public void TestLiftPosition(final XboxController xB)
    {
        final double rate = -xB.getRawAxis(5);
        if( rate < -0.5 || rate > 0.5)
        {
            if(CurrentPos < -60000){
                CurrentPos -= 200*rate;
            }
            else{CurrentPos -= 500*rate;}
        }
        if(CurrentPos < -70830){
            rumblevar = 10;
            CurrentPos = -70830;
        }
        else if(CurrentPos > 0){
            rumblevar = 10;
            CurrentPos = 0;
        }
        ElevatorLeft.set(ControlMode.Position, CurrentPos + currentLeft);
        ElevatorRight.set(ControlMode.Position, CurrentPos + currentRight);
        //System.out.println(ElevatorLeft.getSelectedSensorPosition() + "  " + ElevatorRight.getSelectedSensorPosition() + "  " + CurrentPos + "  " + arm.getSelectedSensorPosition() + "  " + CurrentArmPos);
    }

    public long GetElevatorPosition()
    {
        return FastMath.abs((long)CurrentPos);
    }

    public void ManageRumble(final XboxController xB){
        if(rumblevar > 0){
            xB.setLeftRumble(1);
            xB.setRightRumble(1);
            rumblevar -= 1;
        }
        else{
            xB.setLeftRumble(0);
            xB.setRightRumble(0);
        }
    }
}