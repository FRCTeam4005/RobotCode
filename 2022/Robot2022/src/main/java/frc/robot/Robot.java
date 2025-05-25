
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.armLength;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.SwerveDrive.SwervetankDrive;
import frc.robot.subsystems.limelight.CameraAngles;
import frc.robot.subsystems.limelight.CameraDistance;
import frc.robot.subsystems.limelight.LimeLightVision;
import frc.robot.subsystems.pigeon.Pigeon;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final DigitalInput lazer = new DigitalInput(0);

  public final DriverController Driverinput = new DriverController(0);
  public final OperatorController Operatorinput = new OperatorController(1);
  public final PIDController robot_degrees = new PIDController(0, 0, 0);

  public static enum States
  {
    nothing,
    runIntake,
    forward,
    ready_shoot,
    shoot,
    turn_off
  };


  private States curState = States.nothing;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  double limelightDist = 0;
    double xAngle = 0;
  @Override
  public void robotPeriodic() 
  {

    CameraDistance xyDist = LimeLightVision.instance.getTargetDistance();
    if(xyDist != null)
    {
      limelightDist = xyDist.getForward() / 12;
    }

    CameraAngles xyAngle = LimeLightVision.instance.getCameraAngles();
    if(xyAngle != null)
    {
      xAngle = xyAngle.getTx();
    }

  }


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

  }

  double driveTime = 0;
  int intakeTIme = 0;
  int shoottime = 0;
  @Override
  public void autonomousPeriodic() 
  {
    double shootWait = 0;
    
    switch(curState)
    {
      case nothing:// this does nothing but give the swerve modules time to align
        SwervetankDrive.instance.realign();
        curState = States.runIntake;
        intakeTIme = 0;
      break;

      case runIntake: // this is to make sure the arm is to full length

        intakeTIme++;
        intake.instance.auto_intake(false);
        if(intakeTIme > 20)
        {
          curState = States.forward;
          driveTime = 0;
        }
      break;

      case forward: //drives forwards and continues when a ball is detected
        SwervetankDrive.instance.calculateSwerve(-.1, 0, 0);
        shooter.instance.shootSpeed(3750);
        driveTime++;
        if( driveTime > 280)
        {
          curState = States.ready_shoot;
          shoottime = 0;
        }
      break;

      case ready_shoot:
        intake.instance.auto_intake(false);
        SwervetankDrive.instance.calculateSwerve(0, 0, 0);
        //Turret.instance.runTurret(true, limelightDist);
        shoottime++;
        if(shooter.instance.if_shoot() || shoottime > 50)
        {
          curState = States.shoot;
          shootWait = 0;
        }
      break;

      case shoot:
        elevator.instance.run_elevator(true);
        shootWait++;

        if(shootWait > 25)
        {
          curState = States.turn_off;
        }
      break;

      case turn_off:
        SwervetankDrive.instance.calculateSwerve(0, 0, 0);
        Turret.instance.runTurret(false,0);
        shooter.instance.shootSpeed(0);
        elevator.instance.run_elevator(false);
        intake.instance.auto_intake(false);
      break;
    }
  }

  @Override
  public void teleopInit() 
  {

  }

  @Override
  public void teleopPeriodic() 
  {
    SmartDashboard.putNumber("distance" ,limelightDist);
    SmartDashboard.putNumber("xAngle", xAngle);
    
    SwervetankDrive.instance.calculateSwerve(Driverinput.getFWD(), Driverinput.getSTR(), Driverinput.getRCW());
    
    armLength.instance.run_climber_height(Operatorinput.getLeftStickYAxis());
    LinearActuator.instance.AdjustLinearActuators(Operatorinput.getCenterUp(), Operatorinput.getCenterDown());
    Turret.instance.runTurret(Operatorinput.getLeftTrigger() > .5, xAngle);
    //shooter.instance.test_shoot(Operatorinput.getLeftTrigger() > .5, limelightDist);
    shooter.instance.shoot_at_speed(Operatorinput.getLeftTrigger() > .5, 3000);
    intake.instance.run_intake(Operatorinput.getAButton(), Operatorinput.getyButton());
    Pigeon.instance.reset_pigeon_angle(Driverinput.getYButton());


    if ((Operatorinput.getBButton() && lazer.get()) || Operatorinput.getRightTriggerButton() > .5)
    {
      elevator.instance.run_elevator(true);
    }
    else
    {
      elevator.instance.run_elevator(false);
    }
  }

  @Override
  public void disabledInit() 
  {

  }

  @Override
  public void disabledPeriodic() 
  {

  }

  @Override
  public void testInit() 
  {

  }

  @Override
  public void testPeriodic() 
  {
    intake.instance.run_intake(true, false);
  }
}