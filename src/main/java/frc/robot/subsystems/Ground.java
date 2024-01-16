// akul 
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;


public class Ground extends SubsystemBase {

    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    //private final CANSparkMax intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
    //private final PWMSparkMax intakeMotor = new PWMSparkMax(6);
    private final CANSparkMax intakeMotor = new CANSparkMax(-1 /* CHANGE THIS */, MotorType.kBrushless);


    public void periodic() {
        SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    }

    // turn on the intake motor to pull in objects.
    public void intake() {
        SmartDashboard.putString("Intake Status", "PULL");
        //intakeMotor.setInverted(false);
        intakeMotor.set(1);
    }

    // turn on the intake motor to push out objects.
    public void outtake() {
        SmartDashboard.putString("Intake Status", "PUSH");
        // intakeMotor.setInverted(true);
        intakeMotor.set(-1.0);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Intake Status", "STOP");
        intakeMotor.set(0);
    }

}
