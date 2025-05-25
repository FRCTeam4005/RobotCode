// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;

//import javax.lang.model.util.ElementScanner6;

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
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
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

  private static enum States
  {
    MoveCenterDown,
    SpinUpShooter,
    ShootBall,
    CloseGate,
    FeedBalls,
    TurnOffEverything,
    DriveForward,
    Done

  };

  private States CurrState = States.MoveCenterDown;

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

  CameraDistance distance;
  double shootDist = 0;
  int shootWait = 0;
  int ballCount = 0;
  int initBallWait = 500;
  boolean shootDone = false;
  @Override
  public void autonomousInit() 
  {
    shootWait = 0;
    ballCount = 0;
    shootDone = false;
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
    distance = LimeLightVision.instance.getTargetDistance();
    if(distance != null)
    {
      shootDist = distance.getForward();
    }

    switch(CurrState)
    {
      case MoveCenterDown:
      {
        //checks bottom lineactuator to see where they are and moves them accordingly
        SolenoidPCM.instance.setFeederSolenoid(false);
        if(LinearActuator.instance.getprtBottomSwitch() || LinearActuator.instance.getsbBottomSwitch())
        {
          LinearActuator.instance.AdjustLinearActuators(false, true);
        }
        else
        {
          LinearActuator.instance.AdjustLinearActuators(false, false);
          CurrState = States.SpinUpShooter;
          shootWait = 0;
        }
      }
      break;

      //release ball closest to motor
      //this part actually adds the count for amount of balls shot
      case SpinUpShooter:
        Boolean shooterSpeed = Shooter.instance.Shoot(true, shootDist);
        
        if(shooterSpeed)
        {
          shootWait = 0;
          CurrState = States.ShootBall;
        }
      break;

      case ShootBall:
        Shooter.instance.Shoot(true, shootDist);
        SolenoidPCM.instance.setFeederSolenoid(true);
        shootWait++;
        if(shootWait > 50)
        {
          shootWait = 0;
          ballCount++;
          SolenoidPCM.instance.setFeederSolenoid(false);
          if(ballCount >= 3)
          {
            CurrState = States.TurnOffEverything;
          }
          else
          {
            CurrState = States.FeedBalls;
          }
        }
      break;

      //close solenoid so no other balls fly out
      case FeedBalls:
        Shooter.instance.Shoot(true, shootDist);
        Intake.instance.RunIntake(-1.0);

        shootWait++;
        if(shootWait > 50)
        {
          Intake.instance.RunIntake(0);
          shootWait = 0;
          CurrState = States.SpinUpShooter;  
        }
      break;

    // if ballCount is equal to 3 then set all motor and solinoid to off or default

      case TurnOffEverything:
        Intake.instance.RunIntake(0);
        Shooter.instance.Shoot(0);
        SolenoidPCM.instance.setFeederSolenoid(false);
        shootWait = 0;
        CurrState = States.DriveForward;
      break;

      case DriveForward:
        SwerveDriveTrain.instance.CalculateSwerve(0.35, 0, 0.1);
        
        shootWait++;
        if(shootWait > 75)
        {
          SwerveDriveTrain.instance.CalculateSwerve(0, 0, 0);
          CurrState = States.Done;
        }
      break;

      case Done:
      break;

      default:
        CurrState = States.MoveCenterDown;
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    SwerveDriveTrain.instance.CalculateSwerve(DriverInput.getFWD(), DriverInput.getSTR(), DriverInput.getRCW());
    SwerveDriveTrain.instance.toggleFieldCentric(DriverInput.getToggleFieldCentric());
    
    Intake.instance.RunIntake(OperatorInput.getIntake());
    Shooter.instance.Shoot(OperatorInput.getShoot(), shootDist);
    
    LinearActuator.instance.AdjustLinearActuators(OperatorInput.getCenterUp(), OperatorInput.getCenterDown());
    
    SolenoidPCM.instance.setFeederSolenoid(OperatorInput.getFeeder());
    SolenoidPCM.instance.setLiftSolenoid(OperatorInput.getLift());

    distance = LimeLightVision.instance.getTargetDistance();
    if(distance != null)
    {
      shootDist = distance.getForward();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
