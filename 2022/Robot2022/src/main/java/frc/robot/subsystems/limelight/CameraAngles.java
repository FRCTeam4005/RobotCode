package frc.robot.subsystems.limelight;

public class CameraAngles 
{
  private double tx;
  private double ty;

  public CameraAngles(double tx, double ty) 
  {
    this.tx = tx;
    this.ty = ty;
  }
  
  public double getTx()
  {
    return tx;
  }
  
  public double getTy()
  {
    return ty;
  }
}