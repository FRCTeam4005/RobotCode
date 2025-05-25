package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class intake 
{
    public static final intake instance = new intake(RobotConstants.Intake_spin, RobotConstants.Intake_out);

    private TalonFX Intake_Out;
    private TalonSRX Intake_Spin;

    private intake(int spin, int arm)
    {
        Intake_Out = new TalonFX(arm);
        Intake_Out.configFactoryDefault();
        
        Intake_Out.setNeutralMode(NeutralMode.Brake);
        Intake_Out.setInverted(true);
        Intake_Out.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 50);
        Intake_Out.setSelectedSensorPosition(0);
        
        Intake_Out.selectProfileSlot(0,0);
        Intake_Out.config_kF(0, RobotConstants.F_LINEAR_AC, 50);
        Intake_Out.config_kP(0, RobotConstants.P_LINEAR_AC, 50);
        Intake_Out.config_kI(0, RobotConstants.I_LINEAR_AC, 50);
        Intake_Out.config_kD(0, RobotConstants.D_LINEAR_AC, 50);
        Intake_Out.config_IntegralZone(0, RobotConstants.INT_ZONE_LINEAR_AC);
        Intake_Out.configClosedLoopPeakOutput(0, RobotConstants.PEAK_OUT_LINEAR_AC);
        Intake_Out.configMotionAcceleration(RobotConstants.ACCEL_LINEAR_AC, 50);
        Intake_Out.configMotionCruiseVelocity(RobotConstants.CRUISE_LINEAR_AC, 50);
        Intake_Out.configClosedLoopPeriod(0, 1);
        Intake_Out.set(ControlMode.MotionMagic, 0);
        
        Intake_Spin = new TalonSRX(spin);
    }

    public void run_intake(boolean user_input, boolean spit_ball)
    {
        if (user_input && !spit_ball)
        {
            Intake_Out.set(ControlMode.Position, 330000);
            if (Intake_Out.getSelectedSensorPosition() > 300000)
            {
                Intake_Spin.set(ControlMode.PercentOutput, 0.95);    
            }
            else
            {
                Intake_Spin.set(ControlMode.PercentOutput, 0);
            }

        }
        else if(!user_input && spit_ball)
        {
            Intake_Out.set(ControlMode.Position, 330000);//330000
            if (Intake_Out.getSelectedSensorPosition() > 300000)//300000
            {
                Intake_Spin.set(ControlMode.PercentOutput, -1);    
            }
            else
            {
                Intake_Spin.set(ControlMode.PercentOutput, 0);
            } 
        }
        else
        {
            Intake_Out.set(ControlMode.Position, 0);
            Intake_Spin.set(ControlMode.PercentOutput, 0);
        }

        //SmartDashboard.putNumber("intake position", Intake_Out.getSelectedSensorPosition());
        //SmartDashboard.putNumber("instake speed", Intake_Out.getSelectedSensorVelocity());
    }


    public boolean intakeIfOut()
    {
        if (Intake_Out.getSelectedSensorPosition() >= 30000)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    public double get_voltage()
    {
        //SmartDashboard.putNumber("intake amps", Intake_Spin.getSupplyCurrent());
        return Intake_Spin.getSupplyCurrent();
    }


    public void elevator_help(Boolean user_input)
    {
        if (Intake_Out.getSelectedSensorPosition() < 100 && user_input)
        {
            Intake_Spin.set(ControlMode.PercentOutput, .1);
        }
        else
        {
            Intake_Spin.set(ControlMode.PercentOutput, 0);
        }
    }

    public void auto_intake(boolean on_off)
    {

        if (on_off)
        {
            Intake_Out.set(ControlMode.Position, 330000);
            Intake_Spin.set(ControlMode.PercentOutput, .95);
        }
        else
        {
            Intake_Out.set(ControlMode.Position, 0);
            Intake_Spin.set(ControlMode.PercentOutput, 0);
        }
    }

    
}
