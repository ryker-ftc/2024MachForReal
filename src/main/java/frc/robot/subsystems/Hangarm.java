//neel and ryker
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants.Mod5;

public class Hangarm extends SubsystemBase {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public Hangarm() {
        leftMotor = new CANSparkMax(Mod5.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Mod5.rightMotorID, MotorType.kBrushless);
    }

    public void periodic() {
        
    }

    // extend hangarm
    public void extend() {
        SmartDashboard.putString("Hangarm Status", "EXTEND");
        leftMotor.set(0.01);
        rightMotor.set(0.01);
    }

    // retract hangarm
    public void retract() {
        SmartDashboard.putString("Hangarm Status", "RETRACT");
        leftMotor.set(0.01);
        rightMotor.set(0.01);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Hangarm Status", "STOP");
        leftMotor.set(0);
        rightMotor.set(0);
    }
}