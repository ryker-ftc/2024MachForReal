//neel and ryker
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangarm extends SubsystemBase {
    public void periodic() {
        
    }

    // extend hangarm
    public void extend() {
        SmartDashboard.putString("Hangarm Status", "EXTEND");
        //intakeMotor.setInverted(false);
        // intakeMotor.set(1);
    }

    // retract hangarm
    public void retract() {
        SmartDashboard.putString("Hangarm Status", "RETRACT");
        // intakeMotor.setInverted(true);
        // intakeMotor.set(-1.0);
    }

    // this is the default state of the intake motor - do not move.
    public void stop() {
        SmartDashboard.putString("Hangarm Status", "STOP");
        // intakeMotor.set(0);
    }
}