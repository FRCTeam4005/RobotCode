package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class Turret 
{
    public static final Turret instance = new Turret(RobotConstants.turret_control);
    private CANSparkMax turret_control;
    private PIDController aim_pid;
    private double position;
    private SparkMaxPIDController turret_spark_pid;

   // private static final double MaxPosition = 3.25;

    private Turret(int turret_id)
    {
        turret_control = new CANSparkMax(RobotConstants.turret_control, MotorType.kBrushless);
        turret_control.restoreFactoryDefaults();
        turret_control.setIdleMode(CANSparkMax.IdleMode.kBrake); 

        turret_spark_pid = turret_control.getPIDController();
        //turret_spark_pid.setFF(1);
        turret_spark_pid.setP(.1, 1);
        turret_spark_pid.setI(1e-4 , 1);
        turret_spark_pid.setD(1, 1);
        turret_spark_pid.setOutputRange(-.3, .3);

        
        turret_control.enableSoftLimit(SoftLimitDirection.kForward, true);
        turret_control.enableSoftLimit(SoftLimitDirection.kReverse, true);
        turret_control.setSoftLimit(SoftLimitDirection.kForward, 2);
        turret_control.setSoftLimit(SoftLimitDirection.kReverse, -2);
       

        aim_pid = new PIDController(.01, 0, 0);
    }

    double MovePosition = 0;
    public void runTurret(boolean input, double xAngle)
    {
        position = turret_control.getEncoder().getPosition();

        if(input)
        {
            track(xAngle);
        }
        else 
        {
            set_postion(0.0);
        }

        SmartDashboard.putNumber("move position", MovePosition);
    }

    
    private void aim(double position, double xAngle)
    {

        if(position >= 3 && xAngle > 0)
        {
            set_postion(3.0);
        }
        else if(position >= 3 && xAngle < 0)
        {
            track(xAngle);
        }
        else if(position <= -3 && xAngle < 0)
        {
            set_postion(-3.0);
        }
        else if(position <= -3 && xAngle > 0)
        {
            track(xAngle);
        }

    }
        
    private void track(double xAngle) //(double position)
    {
        //turret_control.set(MaxPosition * position);
        turret_control.set(aim_pid.calculate(xAngle, 0) * .6);
    }
    
    public void set_postion(double position)
    {
        turret_spark_pid.setReference(position, ControlType.kPosition, 1);
        //SmartDashboard.putNumber("turret pos", turret_control.getEncoder().getPosition());
    }

    public double position()
    {
        return turret_control.getEncoder().getPosition();
    }
}
