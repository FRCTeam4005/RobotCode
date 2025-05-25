package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class CompressorPCM
{
  public static final CompressorPCM CompressorInstance = new CompressorPCM(RobotConstants.COMPRESSOR_PCM_CANID);
  private final Compressor _compressor;
  private CompressorPCM(int compCanId)
  {
    _compressor = new Compressor(compCanId);
    _compressor.setClosedLoopControl(SmartDashboard.getBoolean("compressor", true));
  }

  public void CompressorControl(Boolean state)
  {
    _compressor.setClosedLoopControl(state);
  }
}