/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  PigeonIMU _pigIMU;
  TalonSRX _pigTalon = new TalonSRX(2);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    _pigIMU = new PigeonIMU(_pigTalon);

    _pigIMU.setYaw(0, 50);
    _pigIMU.setAccumZAngle(0, 50);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    if(_pigIMU.getState() == PigeonState.Ready)
    {
      double[] ypr = new double[3];
      _pigIMU.getYawPitchRoll(ypr);
      SmartDashboard.putNumber("Yaw: ", ypr[0]);
      SmartDashboard.putNumber("Pitch: ", ypr[1]);
      SmartDashboard.putNumber("Roll: ", ypr[2]);

      double[] quaternions = new double[4];
      _pigIMU.get6dQuaternion(quaternions);
      SmartDashboard.putNumber("Quat W: ", quaternions[0]);
      SmartDashboard.putNumber("Quat X: ", quaternions[1]);
      SmartDashboard.putNumber("Quat Y: ", quaternions[2]);
      SmartDashboard.putNumber("Quat Z: ", quaternions[3]);

      double[] accumGyro = new double[3];
      _pigIMU.getAccumGyro(accumGyro);
      SmartDashboard.putNumber("Accum Gyro X: ", accumGyro[0]);
      SmartDashboard.putNumber("Accum Gyro Y: ", accumGyro[1]);
      SmartDashboard.putNumber("Accum Gyro Z: ", accumGyro[2]);

      short[] biasedAccel = new short[3];
      _pigIMU.getBiasedAccelerometer(biasedAccel);
      SmartDashboard.putNumber("Biased Accel X: ", biasedAccel[0]);
      SmartDashboard.putNumber("Biased Accel Y: ", biasedAccel[1]);
      SmartDashboard.putNumber("Biased Accel Z: ", biasedAccel[2]);

      /** Printing Raw Gyro */
      double[] rawGyro = new double[3];
      _pigIMU.getRawGyro(rawGyro);
      SmartDashboard.putNumber("Raw Gyro X: ", rawGyro[0]);
      SmartDashboard.putNumber("Raw Gyro Y: ", rawGyro[1]);
      SmartDashboard.putNumber("Raw Gyro Z: ", rawGyro[2]);
    
      /** Printing Accelerometer Angles */
      double[] accelAngles = new double[3];
      _pigIMU.getAccelerometerAngles(accelAngles);
      SmartDashboard.putNumber("Accel Angle X: ", accelAngles[0]);
      SmartDashboard.putNumber("Accel Angle Y: ", accelAngles[1]);
      SmartDashboard.putNumber("Accel Angle Z: ", accelAngles[2]);
    
      /** Printing Biased Magnetometer Angles */
      short[] biasedMagnet = new short[3];
      _pigIMU.getBiasedMagnetometer(biasedMagnet);
      SmartDashboard.putNumber("Biased Mag X: ", biasedMagnet[0]);
      SmartDashboard.putNumber("Biased Mag Y: ", biasedMagnet[1]);
      SmartDashboard.putNumber("Biased Mag Z: ", biasedMagnet[2]);
    
      /** Printing Raw Magnetometer Angles */
      short[] rawMagnet = new short[3];
      _pigIMU.getRawMagnetometer(rawMagnet);
      SmartDashboard.putNumber("Raw Mag X: ", rawMagnet[0]);
      SmartDashboard.putNumber("Raw Mag Y: ", rawMagnet[1]);
      SmartDashboard.putNumber("Raw Mag Z: ", rawMagnet[2]);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
