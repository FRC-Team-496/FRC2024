package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Output extends SubsystemBase{
    CANSparkMax m_motor1;
    CANSparkMax m_motor2;
    CANSparkMax m_motor3;
    CANSparkMax m_motor4;
    private double speed = 1;
    SparkMaxPIDController m_pidController1;
    SparkMaxPIDController m_pidController2;
    SparkMaxPIDController m_pidController3;
    SparkMaxPIDController m_pidController4;
    
    public Output(){

        m_motor1 = new CANSparkMax(51, MotorType.kBrushless);
        m_pidController1 = m_motor1.getPIDController();
        m_motor2 = new CANSparkMax(53, MotorType.kBrushless);
        m_pidController2 = m_motor2.getPIDController();
        m_motor3 = new CANSparkMax(50, MotorType.kBrushless);
        m_pidController3 = m_motor3.getPIDController();
        m_motor4 = new CANSparkMax(52, MotorType.kBrushless);
        m_pidController4 = m_motor4.getPIDController();
    }

    // Encoder object created to display position values
    
   
    public void shoot(){
        m_motor1.set(-speed);
        m_motor2.set(speed);
        m_motor3.set(-speed);
        m_motor4.set(speed);
    }

    public void stop(){
        m_motor1.set(0);
        m_motor2.set(0);
        m_motor3.set(0);
        m_motor4.set(0);
    }
}
