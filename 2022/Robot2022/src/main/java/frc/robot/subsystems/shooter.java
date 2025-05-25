package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.subsystems.limelight.LimeLightVision;

public class shooter 
{
    
    public static final shooter instance = new shooter(RobotConstants.left_falcon, RobotConstants.right_falcon);
    
    private TalonFX left_motor;
    private TalonFX right_motor;

    private double Kp = 0.0001;
    private double Ki = 0.0002;
    private double Kd = 0.0;
    private PIDController shooterPID = new PIDController(Kp, Ki, Kd);

    private shooter(int left_falcon, int right_falcon)
    {
        left_motor = new TalonFX(left_falcon);
        left_motor.configFactoryDefault();
        left_motor.setNeutralMode(NeutralMode.Coast);
        left_motor.setInverted(TalonFXInvertType.CounterClockwise);

       right_motor = new TalonFX(right_falcon);
        right_motor.configFactoryDefault();
        right_motor.setNeutralMode(NeutralMode.Coast);
        right_motor.setInverted(TalonFXInvertType.Clockwise);

    }

    double shoot_value;
    public double rpm;

    public void Shoot(double distance, boolean user_input)
    {

        double rpm = ((right_motor.getSelectedSensorVelocity() /2048) * 600);

        if (distance > 2 && user_input)
        {
            shoot_value =  -6.03 * (distance * distance);
            shoot_value += (344.12 * distance) + 735.18;
        }
        else
        {
            shoot_value = 3000;
        }
        SmartDashboard.putNumber("shootvalue", shoot_value);
        left_motor.set(TalonFXControlMode.Velocity, shooterPID.calculate(rpm, shoot_value));
        right_motor.set(TalonFXControlMode.Velocity, shooterPID.calculate(rpm, shoot_value));
    }

    public boolean if_shoot()
    {

        double rpm = ((right_motor.getSelectedSensorVelocity() /2048) * 600);

        if(rpm <= shoot_value + 50 && rpm >= shoot_value - 50)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //avgCount CANNOT BE LESS THAN 3!
    private static final int avgCount = 10;
    double[] rpms = new double[avgCount];
    int rpmCount = 0;
    double avgRPM = 0;

    double alpha = 0.20;
    public boolean test_shoot(boolean input, double distance)
    {
        double shoot_value = 3800;
        double rpm = ((right_motor.getSelectedSensorVelocity() /2048) * 600);
        SmartDashboard.putNumber("shooter velocity", rpm);

        if (input)
        {
            if(distance > 6)
            {
                shoot_value =  -5 * (distance * distance);
                shoot_value += (330.12 * distance) + 735;
            }
            else
            {
                shoot_value = 2900;
            }

            SmartDashboard.putNumber("Shoot Calc", shoot_value);
            left_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, shoot_value));
            right_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, shoot_value));  
        }
        else
        {
            left_motor.set(TalonFXControlMode.PercentOutput, 0);
            right_motor.set(TalonFXControlMode.PercentOutput, 0);  
        }

        if(input)
        {
            avgRPM = (avgRPM * alpha) + (rpm * (1-alpha));
        }
        
        SmartDashboard.putNumber("shooter AVg RPM", avgRPM);


        if(avgRPM >= shoot_value - 100 && input)
        {
            return true;
        }
        else
        {   
            avgRPM = 0;
            return false;
        }
    }

    public void shootSpeed(double speed)
    {
        double rpm = ((right_motor.getSelectedSensorVelocity() /2048) * 600);
        left_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, speed));
        right_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, speed));
        SmartDashboard.putNumber("Auto RPM", rpm);
    }

    public void shooter_off(boolean user_input)
    {
        if(user_input)
        {
            left_motor.set(ControlMode.PercentOutput, 0);
            right_motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void shoot_at_speed(boolean user_input, double speed)
    {
        if(user_input)
        {
        double rpm = ((right_motor.getSelectedSensorVelocity() /2048) * 600);
        left_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, speed));
        right_motor.set(TalonFXControlMode.PercentOutput, shooterPID.calculate(rpm, speed));  
        }
        else
        {
            left_motor.set(TalonFXControlMode.PercentOutput, 0);
            right_motor.set(TalonFXControlMode.PercentOutput, 0);
        }
    }
}
