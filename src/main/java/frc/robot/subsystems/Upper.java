// tej
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Upper extends SubsystemBase {
    /* Config motors and encoders */
    // private final CANSparkMax m_intake = new
    // CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    // private Encoder intakeEncoder = new
    // Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1,
    // Constants.IntakeConstants.Mod4.intakeMotorID2,
    // Constants.IntakeConstants.Mod4.kEncoderReversed);
    private CANSparkMax placementMotor;
    private CANSparkMax flywheelMotor;


    public Upper() {
        placementMotor = new CANSparkMax(-1 /* CHANGE THIS */, MotorType.kBrushless);
        flywheelMotor = new CANSparkMax(-1 /* CHANGE THIS */, MotorType.kBrushless);
    }

    // public void junk() {
    //     lifterMotor.getPIDController().getP();
    //     lifterMotor.getPIDController().getI();
    //     lifterMotor.getPIDController().getD();
    //     SmartDashboard.putNumber("P value", lifterMotor.getPIDController().getP());
    //     SmartDashboard.putNumber("I value", lifterMotor.getPIDController().getI());
    //     SmartDashboard.putNumber("D value", lifterMotor.getPIDController().getD());
    // }

    // turn on the intake motor to pull in objects.
    public void intake() {
        placementMotor.set(-0.25);

        SmartDashboard.putString("Lifter Status", "PUSHING");
    }

    public void ampShoot() {
        // stuff goes here
    }

    public void speakerShoot() {
        placementMotor.set(-0.25);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        lifterMotor.set(0);
        SmartDashboard.putString("Lifter Status", "STOPPED");
    }

    public Object setToPosition(int position) {
        return null;
    }

    public void checkLimits() {
    }
}
