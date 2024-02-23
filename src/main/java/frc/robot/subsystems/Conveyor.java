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
import frc.robot.Constants.ConveyorConstants.Mod4;


public class Conveyor extends SubsystemBase {

    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    //private final CANSparkMax intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
    //private final PWMSparkMax intakeMotor = new PWMSparkMax(6);
    private final CANSparkMax intakeMotor;
    private final CANSparkMax placementMotor;
    private final CANSparkMax flywheelMotorLeft;
    private final CANSparkMax flywheelMotorRight;
    

    public Conveyor() {
        intakeMotor = new CANSparkMax(Mod4.intakeMotorID, MotorType.kBrushless);
        placementMotor = new CANSparkMax(Mod4.placementMotorID, MotorType.kBrushless);
        flywheelMotorLeft = new CANSparkMax(Mod4.flywheelMotorIDLeft, MotorType.kBrushless);
        flywheelMotorRight = new CANSparkMax(Mod4.flywheelMotorIDRight, MotorType.kBrushless);
    }


    public void periodic() {
        SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    }


    public void groundIntake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDINTAKE");
        intakeMotor.set(0.6);
        placementMotor.set(0.2);
    }

    public void groundOuttake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDOUTTAKE");
        intakeMotor.set(-0.5);
        placementMotor.set(-0.2);
    }

    public void upperIntake() {
        SmartDashboard.putString("Conveyor Status", "UPPERINTAKE");
        placementMotor.set(-0.2);
    }
    public void spinUp(double speed){
        SmartDashboard.putString("Conveyor Status", "SPINUP");
        SmartDashboard.putNumber("Conveyor Shoot Speed", speed);
        flywheelMotorLeft.set(-speed);
        flywheelMotorRight.set(-speed);
    }

    public void shoot(double speed) {
        SmartDashboard.putString("Conveyor Status", "SHOOT");
        SmartDashboard.putNumber("Conveyor Shoot Speed", speed);
        placementMotor.set(0.2);
        flywheelMotorLeft.set(-speed);
        flywheelMotorRight.set(-speed);
        
    }
    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Conveyor Status", "STOP");
        intakeMotor.set(0);
        placementMotor.set(0);
        flywheelMotorLeft.set(0);
        flywheelMotorRight.set(0);

    }

}
