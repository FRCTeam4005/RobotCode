package frc.robot.subsystems;

import frc.robot.RobotConstants;
//import frc.robot.controllers.OperatorController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearActuator
{
  public static final LinearActuator instance = new LinearActuator(RobotConstants.PORT_LINEAR_AC, RobotConstants.SB_LINEAR_AC);

  private TalonFX _sbLA;
  private TalonFX _prtLA;

  private static final DigitalInput prtTop = new DigitalInput(0);
  private static final DigitalInput prtBottom = new DigitalInput(1);
  private static final DigitalInput sbTop = new DigitalInput(2);
  private static final DigitalInput sbBottom = new DigitalInput(3);

  public boolean getprtBottomSwitch()
  {
    return prtBottom.get();
  }

  public boolean getsbBottomSwitch()
  {
    return sbBottom.get();
  }

  private LinearActuator(int portCanId, int sbCanId)
  {
    _sbLA = new TalonFX(sbCanId);
    _prtLA = new TalonFX(portCanId);

    

    _sbLA.configFactoryDefault();
    _sbLA.setNeutralMode(NeutralMode.Brake);
    _sbLA.setInverted(false);
    _sbLA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 50);
  
    _prtLA.configFactoryDefault();
    _prtLA.setNeutralMode(NeutralMode.Brake);
    _prtLA.setInverted(false);
    _prtLA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 50);
    
  }

  double Actspeed = 0.65;
  

  public void AdjustLinearActuators(boolean getCenterUp, boolean getCenterDown)
  {

    boolean Topprt= prtTop.get();
    boolean Botprt = prtBottom.get();
    boolean Topsb = sbTop.get();
    boolean botsb = sbBottom.get();

    if (getCenterUp && Topprt) 
    {
      _prtLA.set(ControlMode.PercentOutput, Actspeed);
    }
   else if(getCenterDown && Botprt){
      _prtLA.set(ControlMode.PercentOutput, -Actspeed);
    }
    else
    {
      _prtLA.set(ControlMode.PercentOutput, 0);
    }

    if (getCenterUp && Topsb) 
    {
      _sbLA.set(ControlMode.PercentOutput, Actspeed);
    }
   else if(getCenterDown && botsb){
      _sbLA.set(ControlMode.PercentOutput, -Actspeed);
    }
    else
    {
      _sbLA.set(ControlMode.PercentOutput, 0);
    }

    SmartDashboard.putBoolean("top starboard", Topsb);
    SmartDashboard.putBoolean("bottom starboard", botsb);
    SmartDashboard.putBoolean("top portside", Topprt);
    SmartDashboard.putBoolean("bottom portside", Botprt);
  }

}