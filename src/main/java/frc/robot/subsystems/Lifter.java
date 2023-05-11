package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Lifter extends SubsystemBase {
    /* Config motors and encoders */
    // private final CANSparkMax m_intake = new
    // CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    // private Encoder intakeEncoder = new
    // Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1,
    // Constants.IntakeConstants.Mod4.intakeMotorID2,
    // Constants.IntakeConstants.Mod4.kEncoderReversed);
    private CANSparkMax lifterMotor;

    public Lifter() {
        lifterMotor = new CANSparkMax(14, MotorType.kBrushless);
    }

    // Define the inputs for the limit switches.
    DigitalInput topLimitSwitch = new DigitalInput(0);
    DigitalInput bottomLimitSwitch = new DigitalInput(1);

    private final RelativeEncoder integratedEncoder;
    private final SparkMaxPIDController pidController;

    // This variable defines whether the lift is moving up or down at any given point.
    // UP=1  DOWN=-1
    // This is set to 0 when the lift isn't moving, or when in set-to-position mode.
    private int liftDirection = 0;

    // This variable indicates whether or not we've had a chance to calibrate the internal
    // encoder based on the bottom limit switch yet.  If not, we cannot use the setToPosition mode.
    private boolean calibratedIntegratedEncoder = false;

    // Constructor
    public Lifter() {
        integratedEncoder = lifterMotor.getEncoder();
        pidController = lifterMotor.getPIDController();
    }

    // turn on the intake motor to pull in objects.
    public void push() {
        // lifterMotor.set() configures the motor to run at a constant speed.
        lifterMotor.set(0.25);
        liftDirection = 1;
        SmartDashboard.putString("Lifter Status", "PUSHING");
    }

    // turn on the intake motor to push out objects.
    public void pull() {
        lifterMotor.set(-0.25);
        liftDirection = -1;
        SmartDashboard.putString("Lifter Status", "PULLING");
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        lifterMotor.set(0);
        liftDirection = 0;
        SmartDashboard.putString("Lifter Status", "STOPPED");
    }

    // Run the motor to get to the given position, expressed as a number of rotations of the pulley.
    private void setToPositionRotations(double rotations) {
        // We can only use this mode if the integrated encoder has been calibrated to the limit switch.
        if (calibratedIntegratedEncoder) {
            // Set liftDirection to 0 to indicate that it's being controlled by the PIDController now instead of
            // running it at a constant speed.  This will exempt it from the limit switch checks during this time.
            liftDirection = 0;
            // Set the reference point for the PID controller to the desired number of rotations.  This will cause
            // the pulley to move to that number of rotations.
            pidController.setReference(rotations, ControlType.kPosition);
        }
    }

    // Set to a given position number.  Accepts a parameter of 1, 2, or 3.
    public void setToPosition(int positionNum) {
        // These numbers are the constants for how many times the pulley has to be rotated to get to each position.
        // These probably should be moved to the Constants file for cleanliness, but work here.
        if (positionNum == 1) {
            setToPositionRotations(1.9);
        } else if (positionNum == 2) {
            setToPositionRotations(3.8);
        } else if (positionNum == 3) {
            setToPositionRotations(5.7);
        }
    }

    @Override
    void periodic() {
        // Every tick, make sure we're not hitting limit switches.
        checkLimits();
    }

    public void checkLimits() {
        // Limit switch checks depend on the current direction of movement.  This is required because, for example,
        // if the lift is contacting the bottom limit switch, it needs to be able to move up but not down.
        if (liftDirection == 1 && topLimitSwitch.get()) {
            // We're lifting UP and hit the top limit switch.
            lifterMotor.set(0);
        } else if (liftDirection == -1 && bottomLimitSwitch.get()) {
            // We're going DOWN and hit the bottom limit switch.
            lifterMotor.set(0);
        }
        // Regardless of current direction, if we're hitting the bottom limit switch, use it to calibrate the lifter motor's
        // internal encoder so we can use it for set-to-position.
        // Note that this will continuously recalibrate it for as long as the bottom limit switch is held down.  This isn't a
        // problem, and will ensure that the encoder is zeroed at exactly the point the bottom limit switch is depressed.
        if (bottomLimitSwitch.get()) {
            integratedEncoder.setPosition(0);
            calibratedIntegratedEncoder = true;
        }
    }
}



