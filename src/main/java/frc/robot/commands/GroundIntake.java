package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ground;

public class GroundIntake extends CommandBase {
	Ground m_ground;

    public GroundIntake(Ground m_ground) {
        m_ground = m_ground;
        addRequirements(m_ground);
    }

    @Override
    public void execute() {
        m_ground.pull();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ground.stop();
    }
}
