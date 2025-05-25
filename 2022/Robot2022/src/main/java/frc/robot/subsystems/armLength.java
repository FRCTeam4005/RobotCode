package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class armLength {

    //because they are doing the same thing i am not caring about their names




    double p = .005;
    double i = 0;
    double d = 0;

    public static final armLength instance =  new armLength(RobotConstants.ELEVATOR_MOTOR, RobotConstants.OTHER_ELEVATOR_MOTOR);

    CANSparkMax armMotor;
    CANSparkMax otherArm;
    SparkMaxPIDController armpid;
    SparkMaxPIDController otherArmPid;
    RelativeEncoder armEncoder;
    RelativeEncoder otherArmEncoder;

    private armLength(int motor, int other_motor)
    {
        armMotor = new CANSparkMax(motor, MotorType.kBrushless);
        otherArm = new CANSparkMax(other_motor, MotorType.kBrushless);
        armpid =  armMotor.getPIDController();
        otherArmPid = otherArm.getPIDController();
        armEncoder = armMotor.getEncoder();
        otherArmEncoder = otherArm.getEncoder();

        otherArmPid.setP(p);
        otherArmPid.setI(i);
        otherArmPid.setD(d);

        armpid.setP(p);
        armpid.setI(i);
        armpid.setD(d);

        armpid.setOutputRange(-.7, .7);
        otherArmPid.setOutputRange(-.7, .7);

        armMotor.setInverted(true);
        otherArm.setInverted(true);
    }

    public void run_climber_height(double control)


    {
        control = deadband(control);
        /*
        if(control)
        {
            armpid.setReference(200, ControlType.kPosition);
            otherArmPid.setReference(200, ControlType.kPosition);
        }
        else
        {
            armpid.setReference(0, ControlType.kPosition);
            otherArmPid.setReference(0, ControlType.kPosition);
        }

        SmartDashboard.putNumber("armlength", armEncoder.getPosition());

        */
        armMotor.set(control);
        otherArm.set(control);
    }


    public double deadband(double control)
    {
        if( Math.abs(control) > .07)
        {
            return control;
        }
        else
        {
            return 0;
        }
    }








    
    
}
