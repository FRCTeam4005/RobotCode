package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotConstants;

public class Pigeon
{
  private static final TalonSRX pigeonTalon = new TalonSRX(RobotConstants.PIGEON_TALONID);
  public static final Pigeon instance = new Pigeon(pigeonTalon);
  private static PigeonIMU pigeon;

  Pigeon(TalonSRX talon)
  {
    pigeon = new PigeonIMU(talon);
    pigeon.setFusedHeading(0.0, 50);
  }

  public PigeonData getPigeonData()
  {
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    double [] xyz_dps = new double[3];

    pigeon.getGeneralStatus(genStatus);
    pigeon.getRawGyro(xyz_dps);
    pigeon.getFusedHeading(fusionStatus);

    return new PigeonData(fusionStatus.heading, xyz_dps[2], (pigeon.getState() == PigeonIMU.PigeonState.Ready));
  }
}