//neel and ryker
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangarmConstants.Mod5;



public class Hangarm extends SubsystemBase {

    CANSparkMax leftMotor = new CANSparkMax(Mod5.leftMotorId, MotorType.kBrushless);
    CANSparkMax rightMotor = new CANSparkMax(Mod5.rightMotorId, MotorType.kBrushless);

    public void periodic() {
        
    }

    // extend hangarm
    public void extend() {
        SmartDashboard.putString("Hangarm Status", "EXTEND");
        leftMotor.set(-0.01);
        rightMotor.set(-0.01);
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