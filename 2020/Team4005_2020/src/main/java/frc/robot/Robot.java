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
import frc.robot.controllers.OperatorController;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.limelight.CameraDistance;
import frc.robot.subsystems.limelight.LimeLightVision;
import frc.robot.swerve.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private final DriverController DriverInput = new DriverController(0);
  private final OperatorController OperatorInput = new OperatorController(1);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double fwdDistance;
  private double strDistance;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("Fwd Distance in cm: ", fwdDistance);
    SmartDashboard.putNumber("STR Distance in cm: ", strDistance);
    CompressorPCM.CompressorInstance.CompressorControl(true);
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
  public void autonomousInit() 
  {
    
    //CompressorPCM.CompressorInstance.CompressorControl(false);
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    fwdDistance = SmartDashboard.getNumber("Fwd Distance in cm: ", 0);
    strDistance = SmartDashboard.getNumber("STR Distance in cm: ", 0);
    //double rotAngle = SmartDashboard.getNumber("Rotate Angle in Degrees: ", 0);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    if(Math.abs(fwdDistance) > 0)
    {
      double driveSpeed = 0.15;
      if(fwdDistance < 0)
      {
        driveSpeed = -driveSpeed;
      }
      while(SwerveDriveTrain.instance.getCMDistance() < Math.abs(fwdDistance))
      {
        //FWD
        //Negative is away from starting position
        //Positive is towards starting position
        SwerveDriveTrain.instance.CalculateSwerve(driveSpeed,0,0);
      }
      fwdDistance = 0;
      SwerveDriveTrain.instance.CalculateSwerve(0,0,0);
    }

    //driveTrain.zeroDistanceReading();

    if(Math.abs(strDistance) > 0)
    {
      double driveSpeed = 0.15;
      if(strDistance < 0)
      {
        driveSpeed = -driveSpeed;
      }
      while(SwerveDriveTrain.instance.getCMDistance() < Math.abs(strDistance))
      {
        //FWD
        //Negative is away from starting position
        //Positive is towards starting position
        SwerveDriveTrain.instance.CalculateSwerve(0,driveSpeed,0);
      }
      strDistance = 0;
      SwerveDriveTrain.instance.CalculateSwerve(0,0,0);
    }
    
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
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit()
  {
  }

  int outputPace = 0;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    //if(OperatorInput.getAutoAim() == true)
   // {
      //SwerveDriveTrain.targetAngleFromLimeLight = true;
      //SwerveDriveTrain.instance.CalculateSwerve(DriverInput.getFWD(), DriverInput.getSTR(), 0);
    //}
    //else
    //{
      //SwerveDriveTrain.targetAngleFromLimeLight = false;
      SwerveDriveTrain.instance.CalculateSwerve(DriverInput.getFWD(), DriverInput.getSTR(), DriverInput.getRCW());
      //SwerveDriveTrain.instance.toggleFieldCentric(DriverInput.getToggleFieldCentric());
    //}
    

    Intake.instance.RunIntake(OperatorInput.getIntake());
    Conveyor.instance.RunConveyor(OperatorInput.getConveyor());
    Shooter.instance.Shoot(OperatorInput.getShoot());
    LinearActuator.instance.AdjustLinearActuators(OperatorInput.getCenterUp(), OperatorInput.getCenterDown());
    
    SolenoidPCM.instance.setFeederSolenoid(OperatorInput.getFeeder());
    SolenoidPCM.instance.setLiftSolenoid(OperatorInput.getLift());

    outputPace++;
    if(outputPace > 100)
    {
      outputPace = 0;
      CameraDistance distance = LimeLightVision.instance.getTargetDistance();
      if(distance != null)
      {
        SmartDashboard.putNumber("LLSideDistance:", distance.getSideways());
        SmartDashboard.putNumber("LLFWDDistance:", distance.getForward());
      }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
