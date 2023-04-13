package frc.robot.subsystems;


//import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);

    private final CANSparkMax lifterMotor = new CANSparkMax(14, MotorType.kBrushless);

    // turn on the intake motor to pull in objects.
    public void push() {
        lifterMotor.set(0.25);
        
        SmartDashboard.putString("Lifter Status", "PUSHING");
        SmartDashboard.putNumber("Lifter P Value", lifterMotor.getPIDController().getP());
        SmartDashboard.putNumber("Lifter I Value", lifterMotor.getPIDController().getI());
        SmartDashboard.putNumber("Lifter D Value", lifterMotor.getPIDController().getD());
    }

    // turn on the intake motor to push out objects.
    public void pull() {
        
        DigitalInput maxReach = new DigitalInput(0);

        lifterMotor.set(-0.25);
        SmartDashboard.putString("Lifter Status", "PULLING");
        SmartDashboard.putNumber("Lifter P Value", lifterMotor.getPIDController().getP());
        SmartDashboard.putNumber("Lifter I Value", lifterMotor.getPIDController().getI());
        SmartDashboard.putNumber("Lifter D Value", lifterMotor.getPIDController().getD());
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        lifterMotor.set(0);
        SmartDashboard.putString("Lifter Status", "STOPPED");
        SmartDashboard.putNumber("Lifter P Value", lifterMotor.getPIDController().getP());
        SmartDashboard.putNumber("Lifter I Value", lifterMotor.getPIDController().getI());
        SmartDashboard.putNumber("Lifter D Value", lifterMotor.getPIDController().getD());
    }

    

    public void setToPosition(double distance) {
        //Prevent rotating module if speed is less then 1%. Prevents jittering.
        lifterController.setReference(distance/Constants.LiftConstants.Mod5.kPulleyRatio, ControlType.kPosition);
    }

    /*
    public void setToPosition(JoystickButton button) {
        lifterEncoder.setPosition(0.0);
        if(button.equals(XboxController.Button.kA)) {
            lifterEncoder.setPosition(0.2);           
        } else if(button.equals(XboxController.Button.kB)) {
            lifterEncoder.setPosition(0.3);
        } else if(button.equals(XboxController.Button.kX)) {
            lifterEncoder.setPosition(0.4);
        }
    }
    */

}