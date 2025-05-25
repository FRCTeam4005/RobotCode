package frc.robot.subsystems.SwerveDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxPIDController;



public class SwerveModule 
{

    public CANSparkMax Turn_Motor;
    public CANSparkMax Drive_Motor;

    public SparkMaxPIDController Drive_pid;
    public PIDController Turn_pid;
    public RelativeEncoder drive_encoder;


    public SwerveModule(int Drive_id, boolean Drive_invert, double kp, int Turn_id, boolean Turn_invert)
    {
        this.Turn_Motor = new CANSparkMax(Turn_id, MotorType.kBrushless);
        this.Drive_Motor = new CANSparkMax(Drive_id, MotorType.kBrushless);
        Turn_pid = new PIDController(.01,0,0);

        Drive_pid = Drive_Motor.getPIDController();
        drive_encoder = Drive_Motor.getEncoder();
        drive_encoder.setVelocityConversionFactor(6.75);

        Drive_Motor.setIdleMode(IdleMode.kCoast);
        Drive_Motor.setInverted(Drive_invert);
        Drive_pid.setP(kp ,0);
        Drive_pid.setI(0, 0);
        Drive_pid.setD(0, 0);
        Drive_pid.setFeedbackDevice(drive_encoder);
        Drive_Motor.burnFlash();

        Turn_Motor.restoreFactoryDefaults();
        Turn_Motor.setIdleMode(IdleMode.kBrake);
        Turn_Motor.setInverted(Turn_invert);

    }

    public void drive_Speed (double speed)
    {
        Drive_pid.setReference(speed, ControlType.kVelocity, 0);
    }

    public double distance()
    {
        return Drive_Motor.getEncoder().getPosition() / 21.1475564147;
    }
}
