package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Upper;

public class SpeakerShoot extends CommandBase {
	Upper m_upper;

    public SpeakerShoot(Upper upper) {
        m_upper = upper;
        addRequirements(m_upper);
    }

    @Override
    public void execute() {
        m_upper.speakerShoot();
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
