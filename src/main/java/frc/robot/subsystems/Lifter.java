package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lifter extends SubsystemBase {
    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    private final CANSparkMax lifterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private RelativeEncoder liftRelativeEncoder;

//SmartDashboard.putnumber("P value",lifterm)
    public void defenseX() {
        lifterMotor.getPIDController().setP(0);
        lifterMotor.getPIDController().setI(0);
        lifterMotor.getPIDController().setD(0);

    }

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
}
