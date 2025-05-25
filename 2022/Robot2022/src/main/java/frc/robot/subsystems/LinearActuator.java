package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;


public class LinearActuator {

    public static final LinearActuator instance = new LinearActuator(61, 62);

    private static final DigitalInput prtTop = new DigitalInput(2);
    private static final DigitalInput prtBottom = new DigitalInput(3);

    //private static final DigitalInput sbTop = new DigitalInput(4);
    private static final DigitalInput sbBottom = new DigitalInput(5);

    private TalonFX _sbLA;
    private TalonFX _prtLA;

    private LinearActuator(int portCanId, int sbCanId)
    {
        _sbLA = new TalonFX(sbCanId);
        _prtLA = new TalonFX(portCanId);

        _sbLA.configFactoryDefault();
        _sbLA.setNeutralMode(NeutralMode.Brake);
        _sbLA.setInverted(false);
    
        _prtLA.configFactoryDefault();
        _prtLA.setNeutralMode(NeutralMode.Brake);
        _prtLA.setInverted(false);
    }

    double Actspeed = 0.55;
    public void AdjustLinearActuators(boolean getCenterUp, boolean getCenterDown)
    {
        boolean Topprt= prtTop.get();
        boolean Botprt = prtBottom.get();
        //boolean Topsb = sbTop.get();
        //boolean botsb = sbBottom.get();
    
        if (getCenterUp && Topprt) 
        {
            _prtLA.set(ControlMode.PercentOutput, Actspeed);
        }
        else if(getCenterDown && Botprt)
        {
            _prtLA.set(ControlMode.PercentOutput, -Actspeed);
        }
        else
        {
            _prtLA.set(ControlMode.PercentOutput, 0);
        }
    
        if (getCenterUp && Topprt) 
        {
            _sbLA.set(ControlMode.PercentOutput, Actspeed);
        }
        else if(getCenterDown && Botprt)
        {
            _sbLA.set(ControlMode.PercentOutput, -Actspeed);
        }
        else
        {
            _sbLA.set(ControlMode.PercentOutput, 0);
        }
    }
    
    public void zero()
    {
        boolean Botprt = prtBottom.get();
        boolean botsb = sbBottom.get();

        if (!botsb)
       {
         _prtLA.set(ControlMode.PercentOutput, -.1);
       }
       else
       {
        _prtLA.set(ControlMode.PercentOutput, 0);
       }

       if (!Botprt)
       {
         _sbLA.set(ControlMode.PercentOutput, -.1);
       }
       else
       {
        _sbLA.set(ControlMode.PercentOutput, 0);
       }

    }

}
