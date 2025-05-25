package frc.robot.subsystems;

import frc.robot.RobotConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LinearActuator
{
  public static final LinearActuator instance = new LinearActuator(RobotConstants.PORT_LINEAR_AC, RobotConstants.SB_LINEAR_AC);

  private TalonFX _sbLA;
  private TalonFX _prtLA;

  private enum CenterHeight
  {
    DOWN,
    MIDDLE,
    UP
  }

  private CenterHeight _centerHeight = CenterHeight.DOWN;

  private LinearActuator(int portCanId, int sbCanId)
  {
    _sbLA = new TalonFX(sbCanId);
    _prtLA = new TalonFX(portCanId);

    _sbLA.configFactoryDefault();
    _sbLA.setNeutralMode(NeutralMode.Brake);
    _sbLA.setInverted(false);
    _sbLA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 50);
    _sbLA.setSelectedSensorPosition(0);
    _sbLA.selectProfileSlot(0,0);
    _sbLA.config_kF(0, RobotConstants.F_LINEAR_AC, 50);
    _sbLA.config_kP(0, RobotConstants.P_LINEAR_AC, 50);
    _sbLA.config_kI(0, RobotConstants.I_LINEAR_AC, 50);
    _sbLA.config_kD(0, RobotConstants.D_LINEAR_AC, 50);
    _sbLA.config_IntegralZone(0, RobotConstants.INT_ZONE_LINEAR_AC);
    _sbLA.configClosedLoopPeakOutput(0, RobotConstants.PEAK_OUT_LINEAR_AC);
    _sbLA.configMotionAcceleration(RobotConstants.ACCEL_LINEAR_AC, 50);
    _sbLA.configMotionCruiseVelocity(RobotConstants.CRUISE_LINEAR_AC, 50);
    _sbLA.configClosedLoopPeriod(0, 1);
    _sbLA.set(ControlMode.MotionMagic, 0);
  
    _prtLA.configFactoryDefault();
    _prtLA.setNeutralMode(NeutralMode.Brake);
    _prtLA.setInverted(false);
    _prtLA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 50);
    _prtLA.setSelectedSensorPosition(0);
    _prtLA.selectProfileSlot(0,0);
    _prtLA.config_kF(0, RobotConstants.F_LINEAR_AC, 50);
    _prtLA.config_kP(0, RobotConstants.P_LINEAR_AC, 50);
    _prtLA.config_kI(0, RobotConstants.I_LINEAR_AC, 50);
    _prtLA.config_kD(0, RobotConstants.D_LINEAR_AC, 50);
    _prtLA.config_IntegralZone(0, RobotConstants.INT_ZONE_LINEAR_AC);
    _prtLA.configClosedLoopPeakOutput(0, RobotConstants.PEAK_OUT_LINEAR_AC);
    _prtLA.configMotionAcceleration(RobotConstants.ACCEL_LINEAR_AC, 50);
    _prtLA.configMotionCruiseVelocity(RobotConstants.CRUISE_LINEAR_AC, 50);
    _prtLA.configClosedLoopPeriod(0, 1);
    _prtLA.set(ControlMode.MotionMagic, 0);
  }

  boolean upcommandRun = false;
  boolean downcommandRun = false;
  public void AdjustLinearActuators( boolean upInput, boolean downInput )
  {
    if(upInput && upcommandRun == false)
    {
      IncreaseCenterHeight();
      upcommandRun = true;
    }
    else if(!upInput)
    {
      upcommandRun = false;
    }

    if(downInput && downcommandRun == false)
    {
      DecreaseCenterHeight();
      downcommandRun = true;
    }
    else if(!downInput)
    {
      downcommandRun = false;
    }

    switch(_centerHeight)
    {
      case DOWN:
        LiftControlDown();
        break;
      case MIDDLE:
        LiftControlMiddle();
        break;
      case UP:
        LiftControlUp();
        break;
      default:
        LiftControlDown();
        break;
    }
  }

  private void IncreaseCenterHeight()
  {
    switch( _centerHeight )
    {
      case DOWN:
        _centerHeight = CenterHeight.MIDDLE;
        break;
      case MIDDLE:
        _centerHeight = CenterHeight.UP;
        break;
      default:
        break;
    }
  }

  private void DecreaseCenterHeight()
  {
    switch( _centerHeight )
    {
      case UP:
        _centerHeight = CenterHeight.MIDDLE;
        break;
      case MIDDLE:
        _centerHeight = CenterHeight.DOWN;
        break;
      default:
        break;
    }
  }

  public void LiftControlDown()
  {
    _prtLA.set(ControlMode.Position, RobotConstants.DOWN_POSITION_LINEAR_AC);
    _sbLA.set(ControlMode.Position, RobotConstants.DOWN_POSITION_LINEAR_AC);
  }

  public void LiftControlUp()
  {
    _prtLA.set(ControlMode.Position, RobotConstants.UP_POSITION_LINEAR_AC);
    _sbLA.set(ControlMode.Position, RobotConstants.UP_POSITION_LINEAR_AC);
  }

  public void LiftControlMiddle()
  {
    _prtLA.set(ControlMode.Position, RobotConstants.MIDDLE_POSITION_LINEAR_AC);
    _sbLA.set(ControlMode.Position, RobotConstants.MIDDLE_POSITION_LINEAR_AC);
  }
}