package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Upper;

public class UpperIntake extends CommandBase {
	Upper m_upper;

    public UpperIntake(Upper upper) {
        m_upper = upper;
        addRequirements(m_upper);
    }

    @Override
    public void execute() {
        m_upper.intake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_upper.stop();
    }
}
