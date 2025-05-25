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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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

  private final XboxController TestController = new XboxController(0);
  private final TalonFX LeftActuator = new TalonFX(20);

  double f_ = 0;
  double p_ = 0.65;
  double i_ = 0.0001;
  double d_ = 0;
  int iZone = 500;
  double peakOut = 1;
  int accel = 3000;
  int cruise = 4000;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("p", p_);
    SmartDashboard.putNumber("i", i_);
    SmartDashboard.putNumber("d", d_);
    SmartDashboard.putNumber("f", f_);
    SmartDashboard.putNumber("izone", iZone);
    SmartDashboard.putNumber("peakout", peakOut);
    SmartDashboard.putNumber("accel", accel);
    SmartDashboard.putNumber("cruise", cruise);
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

  double position = 0;

  @Override
  public void teleopInit()
  {

    p_ = SmartDashboard.getNumber("p", p_);
    i_ = SmartDashboard.getNumber("i", i_);
    d_ = SmartDashboard.getNumber("d", d_);
    f_ = SmartDashboard.getNumber("f", f_);
    iZone = (int)SmartDashboard.getNumber("izone", iZone);
    peakOut = SmartDashboard.getNumber("peakout", peakOut);
    accel = (int)SmartDashboard.getNumber("accel", accel);
    cruise = (int)SmartDashboard.getNumber("cruise", cruise);
    
    

    LeftActuator.configFactoryDefault();
    LeftActuator.setNeutralMode(NeutralMode.Brake);
    LeftActuator.setInverted(false);
    LeftActuator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
    LeftActuator.setSelectedSensorPosition(0);
    LeftActuator.selectProfileSlot(0,0);
    LeftActuator.config_kF(0, f_, 50);
    LeftActuator.config_kP(0, p_, 50);
    LeftActuator.config_kI(0, i_, 50);
    LeftActuator.config_kD(0, d_, 50);
    LeftActuator.config_IntegralZone(0, iZone);
    LeftActuator.configClosedLoopPeakOutput(0, peakOut);
    LeftActuator.configMotionAcceleration(accel, 50);
    LeftActuator.configMotionCruiseVelocity(cruise, 500);
    LeftActuator.configClosedLoopPeriod(0, 1);
    position = 0;
    LeftActuator.set(ControlMode.MotionMagic, 0);
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {

    if(TestController.getXButton())
    {
      position = 0;
      LeftActuator.set(ControlMode.MotionMagic, position);
    }

    if(TestController.getAButton())
    {
      position = 500;
      LeftActuator.set(ControlMode.MotionMagic, position);
    }

    if(TestController.getYButton())
    {
      position = 120000;
      LeftActuator.set(ControlMode.MotionMagic, position);
    }

    if(TestController.getBButton())
    {
      position = 240000;
      LeftActuator.set(ControlMode.MotionMagic, position);
    }

    if(Math.abs(TestController.getLeftStickYAxis()) > 0.10)
    {
      LeftActuator.set(ControlMode.PercentOutput, TestController.getLeftStickYAxis());
    }

    SmartDashboard.putNumber("Position : ", LeftActuator.getSelectedSensorPosition());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
