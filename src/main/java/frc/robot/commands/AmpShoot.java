package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Upper;

public class AmpShoot extends CommandBase {
	Upper m_upper;

    public AmpShoot(Upper upper) {
        m_upper = upper;
        addRequirements(m_upper);
    }

    @Override
    public void execute() {
        m_upper.ampShoot();
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
