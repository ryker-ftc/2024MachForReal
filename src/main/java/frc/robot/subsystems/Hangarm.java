//neel and ryker
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangarmConstants.Mod5;



public class Hangarm extends SubsystemBase {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private final double RETRACTED_POS = 0;
    private final double EXTENDED_POS = 0;


    public Hangarm() {
        leftMotor = new CANSparkMax(Mod5.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Mod5.rightMotorID, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    }

    public void periodic() {
        SmartDashboard.putNumber("left Pos", leftEncoder.getPosition() );
        SmartDashboard.putNumber("right Pos", rightEncoder.getPosition() );
    }

    // extend hangarm
    public void extend() {
        SmartDashboard.putString("Hangarm Status", "EXTEND");
        if (leftEncoder.getPosition() < EXTENDED_POS)leftMotor.set(-0.5);
        if (leftEncoder.getPosition() < EXTENDED_POS)rightMotor.set(0.5);
    }

    // retract hangarm
    public void retract() {
        SmartDashboard.putString("Hangarm Status", "RETRACT");
        if (leftEncoder.getPosition() < RETRACTED_POS) leftMotor.set(0.5);
        if (rightEncoder.getPosition() < RETRACTED_POS) rightMotor.set(-0.5);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Hangarm Status", "STOP");
        leftMotor.set(0);
        rightMotor.set(0);
    }
}