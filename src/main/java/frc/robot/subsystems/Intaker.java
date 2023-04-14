package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.IntakeConstants;


public class Intaker extends SubsystemBase {

    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID, MotorType.kBrushless);

    // turn on the intake motor to pull in objects.
    public void pull() {
        SmartDashboard.putString("Intake Status", "PULL");
        intakeMotor.set(0.25);
    }

    // turn on the intake motor to push out objects.
    public void push() {
        intakeMotor.set(-0.25);
        SmartDashboard.putString("Intake Status", "PUSH");
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        intakeMotor.set(0);
        SmartDashboard.putString("Intake Status", "STOP");
    }


}
