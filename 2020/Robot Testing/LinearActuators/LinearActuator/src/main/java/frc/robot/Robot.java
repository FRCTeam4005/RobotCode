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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private final XboxController TestController = new XboxController(0);
  private final TalonSRX LeftActuator = new TalonSRX(2);
  private final TalonSRX RightActuator = new TalonSRX(9);
  private final SensorCollection LASensor = new SensorCollection(LeftActuator);
  private final SensorCollection RASensor = new SensorCollection(RightActuator);
  //private final SensorCollection 
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    LeftActuator.configFactoryDefault();
    RightActuator.configFactoryDefault();
    LeftActuator.setNeutralMode(NeutralMode.Brake);
    RightActuator.setNeutralMode(NeutralMode.Brake);


    LeftActuator.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    RightActuator.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    LeftActuator.configForwardSoftLimitThreshold(600);
    LeftActuator.configForwardSoftLimitEnable(true);

    LeftActuator.configReverseSoftLimitThreshold(550);
    LeftActuator.configReverseSoftLimitEnable(true);

    RightActuator.configForwardSoftLimitThreshold(910);
    RightActuator.configForwardSoftLimitEnable(true);
    
    RightActuator.configReverseSoftLimitThreshold(850);
    RightActuator.configForwardSoftLimitEnable(true);
    
    //Actuator all the way closed 1.1v  387 raw
    //Actuator all the way open 600 raw
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
    double leftControl = TestController.getLeftStickYAxis();
    double rightControl = TestController.getRightStickYAxis();
    double leftCounts = LASensor.getAnalogIn();
    double rightCounts = RASensor.getAnalogIn();

    LeftActuator.set(ControlMode.PercentOutput, leftControl);
    RightActuator.set(ControlMode.PercentOutput, rightControl);

    SmartDashboard.putNumber("A2D Left : ", leftCounts );
    SmartDashboard.putNumber("A2D Right : ", rightCounts );

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
/*
  public void accurateShooter(XboxController joystick){
    // Aiming and Range at the same time
	float KpAim = -0.1f;
	float KpDistance = -0.1f;
	float min_aim_command = 0.05f;

	double txValue = tx.getDouble(0.0);
	double tyValue = ty.getDouble(0.0);
	if (joystick.getAButton())
	{
		double heading_error = -txValue;
		double distance_error = -tyValue;
		double steering_adjust = 0.0f;

		if (txValue > 1.0)
		{
				steering_adjust = KpAim*heading_error - min_aim_command;
		}
		else if (txValue < 1.0)
		{
				steering_adjust = KpAim*heading_error + min_aim_command;
		}

		double distance_adjust = KpDistance * distance_error;

		left_command += steering_adjust + distance_adjust;
		right_command -= steering_adjust + distance_adjust;
	}
  }
  */
}
