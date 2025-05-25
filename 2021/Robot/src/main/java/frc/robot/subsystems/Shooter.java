package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.*;
import frc.robot.RobotConstants;

public class Shooter
{
  public static final Shooter instance = new Shooter(RobotConstants.SHOOTER_TALONID);
  private TalonFX shooterControl;
  private double targetRPM = 3000;
  private double Kp = 0.0001;
  private double Ki = 0.0002;
  private double Kd = 0.0;
  private PIDController shooterPID = new PIDController(Kp, Ki, Kd);

  private Shooter(int shooterId)
  {
    shooterControl = new TalonFX(shooterId);
    shooterControl.configFactoryDefault();
    shooterControl.setNeutralMode(NeutralMode.Coast);
    shooterControl.setInverted(TalonFXInvertType.Clockwise);
  }

  double ShooterSpeed = 0;
  int speedOutputCount = 0;
  int rpmCount = 0;
  public Boolean Shoot(Boolean state, double distance)
  {
    Boolean atSpeed = false;
    int sensorVel = shooterControl.getSelectedSensorVelocity();
    double rpms = ((double)(sensorVel) / 2048) * 600;
    SmartDashboard.putNumber("Shooter RPM: ", rpms);
    SmartDashboard.putNumber("Shooter Dist", distance);
    if(distance < 100)
    {
      targetRPM = 3000;
    }
    else if(distance >= 110 && distance <= 185)
    {
      targetRPM = 2750;
    }
    else if(distance > 200)
    {
      targetRPM = 2850;
    }

    if(state == true)
    {
      ShooterSpeed = shooterPID.calculate(rpms, targetRPM);
      shooterControl.set(ControlMode.PercentOutput, ShooterSpeed);
    }
    else
    {
      ShooterSpeed = 0;
      shooterPID.calculate(rpms, 0);
      shooterControl.set(ControlMode.PercentOutput, ShooterSpeed);
    }

    rpmCount++;
    if(rpmCount > 12)
    {
      rpmCount=0;
    }

    if( rpms >= (targetRPM * 0.95) )
    {
      atSpeed = true;
    }

    speedOutputCount++;
    if(speedOutputCount > 100)
    {
      SmartDashboard.putNumber("targetRPM:", targetRPM);
      speedOutputCount = 0;
      shooterPID.setP(Kp);
      shooterPID.setI(Ki);
      shooterPID.setD(Kd);
    }

    return atSpeed;
  }

  public void Shoot(double shootValue)
  {
    shooterControl.set(ControlMode.PercentOutput, shootValue);
  }
}
