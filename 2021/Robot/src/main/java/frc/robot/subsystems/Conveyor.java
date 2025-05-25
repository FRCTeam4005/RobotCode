package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class Conveyor
{
  public static final Conveyor instance = new Conveyor(RobotConstants.CONVEYOR_TALONID);
  private VictorSPX conveyorControl;

  private double conveyorSpeed = 0;
  private double conveyorAvg = 0;
  private int outputSpeedCnt = 0;
  private Conveyor(int convId)
  {
    conveyorControl = new VictorSPX(convId);
  }

  public void RunConveyor(double runVal)
  {
    outputSpeedCnt++;
    if(outputSpeedCnt > 100)
    {
      outputSpeedCnt = 0;
      SmartDashboard.putNumber("Avg. conveyor Value: ", conveyorAvg);
    }

    conveyorSpeed = 0;
    if(Math.abs(runVal) > 0.05)
    {
      conveyorSpeed = runVal;
      conveyorAvg += conveyorAvg;
      conveyorAvg /= 2;
    }

    conveyorControl.set(ControlMode.PercentOutput, conveyorSpeed);
  }
}