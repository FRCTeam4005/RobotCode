
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SwerveModule
{
    public TalonSRX DriveMotor;
    public TalonSRX TurnMotor;

    public SwerveModule( int turnMotorID, Boolean reverseTurn, int driveMotorID, Boolean reverseDrive)
    {
        this.DriveMotor = new TalonSRX(driveMotorID);
        this.TurnMotor = new TalonSRX(turnMotorID);

        DriveMotor.configFactoryDefault();
        DriveMotor.set(ControlMode.PercentOutput,0);
        DriveMotor.setNeutralMode(NeutralMode.Brake);
        DriveMotor.configOpenloopRamp(0.75);
        
        TurnMotor.configFactoryDefault();
        TurnMotor.setNeutralMode(NeutralMode.Brake);
        TurnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        TurnMotor.setSelectedSensorPosition(0);
        TurnMotor.selectProfileSlot(0,0);
        TurnMotor.config_kF(0, 0, 50);
        TurnMotor.config_kP(0, 30, 50);
        TurnMotor.config_kI(0, 0.00001, 50);
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

    public void SwerveModuleInit(TalonSRX driveMotor, TalonSRX turnMotor )
    {
        driveMotor.configFactoryDefault();
        driveMotor.set(ControlMode.PercentOutput,0);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        
        turnMotor.configFactoryDefault();
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turnMotor.setSelectedSensorPosition(0);
        turnMotor.selectProfileSlot(0,0);
        turnMotor.config_kF(0, 0, 50);
        turnMotor.config_kP(0, 30, 50);
        turnMotor.config_kI(0, 0.00001, 50);
        turnMotor.config_kD(0, 0.8, 50);
    }
}