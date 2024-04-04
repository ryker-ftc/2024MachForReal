// akul 
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TCS34725ColorSensor;
import frc.lib.TCS34725ColorSensor.TCSColor;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ConveyorConstants.Mod4;


public class Conveyor extends SubsystemBase {

    /*Config motors and encoders */
    //private final CANSparkMax m_intake = new CANSparkMax(Constants.IntakeConstants.Mod4.intakeMotorID,MotorType.kBrushless);
    //private Encoder intakeEncoder = new Encoder(Constants.IntakeConstants.Mod4.intakeMotorID1, Constants.IntakeConstants.Mod4.intakeMotorID2, Constants.IntakeConstants.Mod4.kEncoderReversed);
    //private final CANSparkMax intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
    //private final PWMSparkMax intakeMotor = new PWMSparkMax(6);
    private final CANSparkMax intakeMotorTop;
    private final CANSparkMax intakeMotorBottom;
    private final CANSparkMax placementMotor;
    private final CANSparkMax flywheelMotorLeft;
    private final CANSparkMax flywheelMotorRight;
    public TCS34725ColorSensor colorSensor;
    private final TCSColor noteColor = new TCSColor(0, 0, 0, 0);

    public Conveyor() {
        intakeMotorTop = new CANSparkMax(Mod4.intakeMotorIDTop, MotorType.kBrushless);
        intakeMotorBottom = new CANSparkMax(Mod4.intakeMotorIDBottom, MotorType.kBrushless);
        placementMotor = new CANSparkMax(Mod4.placementMotorID, MotorType.kBrushless);
        flywheelMotorLeft = new CANSparkMax(Mod4.flywheelMotorIDLeft, MotorType.kBrushless);
        flywheelMotorRight = new CANSparkMax(Mod4.flywheelMotorIDRight, MotorType.kBrushless);
        colorSensor = new TCS34725ColorSensor();
        colorSensor.init();
    }


    public void periodic() {
        // SmartDashboard.putNumber("Intake speed", intakeMotor.get());
        SmartDashboard.putNumber("Speed", flywheelMotorLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("red COLORSENSOR", colorSensor.readColors().getR());
        SmartDashboard.putNumber("green COLORSENSOR", colorSensor.readColors().getG());
        SmartDashboard.putNumber("blue COLORSENSOR", colorSensor.readColors().getB());
        SmartDashboard.putNumber("C COLORSENSOR", colorSensor.readColors().getC());
        


    }


    public void groundIntake() {
        // TCSColor color = colorSensor.readColors();
        
        SmartDashboard.putString("Conveyor Status", "GROUNDINTAKE");
        // if (!color.equals(noteColor)){
        //     intakeMotor.set(1);
        //     placementMotor.set(0.4);
        // }
        intakeMotorTop.set(0.7);
        intakeMotorBottom.set(-0.7);
        placementMotor.set(0.7);
    }

    public void groundOuttake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDOUTTAKE");
        intakeMotorTop.set(-0.5);
        intakeMotorBottom.set(0.5);
        placementMotor.set(-0.2);
    }

    public void upperIntake() {
        SmartDashboard.putString("Conveyor Status", "UPPERINTAKE");
        placementMotor.set(-0.2);
    }
    public void spinUp(double speed){
        SmartDashboard.putString("Conveyor Status", "SPINUP");
        SmartDashboard.putNumber("Conveyor Shoot Speed", speed);
        flywheelMotorLeft.set(-1);
        flywheelMotorRight.set(1);

    }

    public void shoot(double speed) {
        SmartDashboard.putString("Conveyor Status", "SHOOT");
        SmartDashboard.putNumber("Conveyor Shoot Speed", speed);
        intakeMotorTop.set(0.8);
        intakeMotorBottom.set(-0.8);
        placementMotor.set(0.8);
        flywheelMotorLeft.set(-1);
        flywheelMotorRight.set(1);
        
    }
    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Conveyor Status", "STOP");
        intakeMotorTop.set(0);
        intakeMotorBottom.set(0);
        placementMotor.set(0);
        flywheelMotorLeft.set(0);
        flywheelMotorRight.set(0);

    }

    public void setIntakeMotorTop(double value) {
        intakeMotorTop.set(value);
    }

    public void setIntakeMotorBottom(double value) {
        intakeMotorBottom.set(value);
    }

    public void setPlacementMotor(double value) {
        placementMotor.set(value);
    }

    public void setFlywheelMotorLeft(double value) {
        flywheelMotorLeft.set(value);
    }

    public void setFlywheelMotorRight(double value) {
        flywheelMotorRight.set(value);
    }

}
