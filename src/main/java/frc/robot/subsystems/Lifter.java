package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter {
    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    private final CANSparkMax lifterMotor = new CANSparkMax(14, MotorType.kBrushless);

    // turn on the intake motor to pull in objects.
    public void push() {
        lifterMotor.set(0.25);
        SmartDashboard.putString("Lifter Status", "PUSHING");
    }

    // turn on the intake motor to push out objects.
    public void pull() {
        lifterMotor.set(-0.25);
        SmartDashboard.putString("Lifter Status", "PULLING");
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        lifterMotor.set(0);
        SmartDashboard.putString("Lifter Status", "STOPPED");
    }
}
