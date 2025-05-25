package frc.robot.subsystems;

public class PigeonData
{
  public double fusionangle;
  public double angularrate;
  public boolean goodangle; 

  PigeonData(double fusionAngle, double angularRate, boolean goodAngle)
  {
    this.fusionangle = fusionAngle;
    this.angularrate = angularRate;
    this.goodangle = goodAngle;
  }
}