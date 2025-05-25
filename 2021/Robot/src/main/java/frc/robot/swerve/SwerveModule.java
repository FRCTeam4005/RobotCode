
package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule
{
    public TalonFX DriveMotor;
    public TalonSRX TurnMotor;

    public SwerveModule( int turnMotorID, Boolean reverseTurn, int driveMotorID, Boolean reverseDrive)
    {
        this.DriveMotor = new TalonFX(driveMotorID);
        this.TurnMotor = new TalonSRX(turnMotorID);

        DriveMotor.configFactoryDefault();
        DriveMotor.set(ControlMode.PercentOutput,0);
        DriveMotor.setNeutralMode(NeutralMode.Brake);
        DriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        DriveMotor.setSelectedSensorPosition(0);
        
        
        TurnMotor.configFactoryDefault();
        TurnMotor.setNeutralMode(NeutralMode.Brake);
        TurnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        TurnMotor.setSelectedSensorPosition(0);
        TurnMotor.selectProfileSlot(0,0);
        TurnMotor.config_kF(0, 0, 50);
        TurnMotor.config_kP(0, 5, 50);
        TurnMotor.config_kI(0, 0.001, 50);
        TurnMotor.config_kD(0, 0.8, 50);
        

        if( reverseDrive )
        {
            DriveMotor.setInverted(true);
        }

        if( reverseTurn )
        {
            TurnMotor.setInverted(true);
        }
    }
}