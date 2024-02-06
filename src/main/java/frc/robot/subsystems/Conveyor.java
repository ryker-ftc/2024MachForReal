// akul 
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;


public class Conveyor extends SubsystemBase {

    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    //private final CANSparkMax intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
    //private final PWMSparkMax intakeMotor = new PWMSparkMax(6);
    private final CANSparkMax intakeMotor;
    private final CANSparkMax placementMotor;
    private final CANSparkMax flywheelMotor1;
    private final CANSparkMax flywheelMotor2;
    

    public Conveyor() {
        intakeMotor = new CANSparkMax(ConveyorConstants.Mod4.intakeMotorID /* CHANGE THIS */, MotorType.kBrushless);
        placementMotor = new CANSparkMax(ConveyorConstants.Mod4.placementMotorID /* CHANGE THIS */, MotorType.kBrushless);
        flywheelMotor = new CANSparkMax(ConveyorConstants.Mod4.flywheelMotorID /* CHANGE THIS */, MotorType.kBrushless);
    }


    public void periodic() {
        SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    }


    public void groundIntake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDINTAKE");
        intakeMotor.set(0.1);
        placementMotor.set(0.1);
    }

    public void groundOuttake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDOUTTAKE");
        intakeMotor.set(-0.1);
        placementMotor.set(-0.1);
    }

    public void upperIntake() {
        SmartDashboard.putString("Conveyor Status", "UPPERINTAKE");
        placementMotor.set(-0.1);
    }

    public void shoot(double speed) {
        SmartDashboard.putString("Conveyor Status", "SHOOT");
        SmartDashboard.putNumber("Conveyor Shoot Speed", speed);
        placementMotor.set(0.1);
        flywheelMotor1.set(speed);
        flywheelMotor2.set(-speed);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Conveyor Status", "STOP");
        intakeMotor.set(0);
        placementMotor.set(0);
        flywheelMotor1.set(0);
        flywheelMotor2.set(0);

    }

}
