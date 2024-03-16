package frc.robot.subsystems.amper;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amper extends SubsystemBase{
    private Servo m_servo1;
    private Servo m_servo2;

    public Amper(){
        m_servo1 = new Servo(0);
        m_servo2 = new Servo(0);
        m_servo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_servo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    public void setServo(double a, double b){
        m_servo1.set(a);
        m_servo2.set(b);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Servo 1 pos", m_servo1.get());
        SmartDashboard.putNumber("Servo 2 pos", m_servo2.get());
    }


}
