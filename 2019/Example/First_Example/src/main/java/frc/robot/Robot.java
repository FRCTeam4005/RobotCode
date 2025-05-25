/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot 
{
  private final XboxController xBox = new XboxController(0);
  private final XboxController xBox2 = new XboxController(1);

  private SwerveDriveTrain driveTrain;
  private Elevator elevator;

  private boolean isElevGoing = false;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(320, 240);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Video_1", 160, 120);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
    }).start();
  
    driveTrain = SwerveDriveTrain.getInstance();
    elevator = Elevator.getInstance();
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() 
  {
    elevator.ElevatorInit();
    isElevGoing = true;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() 
  {
    if(!isElevGoing){
      elevator.ElevatorInit();
      isElevGoing = true;
    }
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() 
  {
    long elevatorPos = elevator.GetElevatorPosition();
    driveTrain.CalculateSwerve( xBox.getLeftStickYAxis(), xBox.getLeftStickXAxis(), xBox.getRightStickXAxis(), elevatorPos );
    elevator.TestLiftPosition(xBox2);
    elevator.TestArmLift(xBox2);
    elevator.TestIntake(xBox2);
    elevator.ManageRumble(xBox2);

    //TAKE THIS OUT FOR COMPETITION
    //elevator.TuneLiftRopesLEFT(xBox2);
    //elevator.TuneLiftRopesRIGHT(xBox2);
    
  }

  @Override
  public void disabledPeriodic() 
  {
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}