package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotConstants;

public class elevator 
{
    public static final elevator instance = new elevator(RobotConstants.elevator_control);

    private TalonFX elevator_motor;

    private elevator(int elevator_id)
    {
        elevator_motor = new TalonFX(elevator_id);
        elevator_motor.setNeutralMode(NeutralMode.Brake);
        elevator_motor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void run_elevator(boolean user_input)
    {
       if (user_input)
       {
           elevator_motor.set(TalonFXControlMode.PercentOutput, RobotConstants.elevator_speed);
       }
       else
       {
           elevator_motor.set(TalonFXControlMode.PercentOutput, 0);
       }
        
    }

    

}
