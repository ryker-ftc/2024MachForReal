package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardWrapper {

    public ShuffleboardWrapper(SendableChooser<String> chooser) {
        Shuffleboard.getTab("Autonomous")
                .add("Autonomous Mode", chooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }
}
