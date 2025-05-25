package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class Intake
{
  public static final Intake instance = new Intake(RobotConstants.INTAKE_TALONID);
  private VictorSPX intakeControl;

  private double intakeSpeed = 0;
  private double intakeAvg = 0;
  private int outputSpeedCnt = 0;
  private Intake(int intakeCANId)
  {
    intakeControl = new VictorSPX(intakeCANId);
  }

  public void RunIntake(double runVal)
  {
    outputSpeedCnt++;
    if(outputSpeedCnt > 100)
    {
      outputSpeedCnt = 0;
      SmartDashboard.putNumber("Avg. Intake Value: ", intakeAvg);
    }

    intakeSpeed = 0;
    if(Math.abs(runVal) > 0.05)
    {
      intakeSpeed = runVal;
      intakeAvg += intakeSpeed;
      intakeAvg /= 2;
    }

    intakeControl.set(ControlMode.PercentOutput, intakeSpeed);
  }   
}
