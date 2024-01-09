package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hangarm;

public class HangarmUp extends CommandBase {
	Hangarm m_hangarm;

    public HangarmUp(Hangarm hangarm) {
        m_hangarm = hangarm;
        addRequirements(m_hangarm);
    }

    @Override
    public void execute() {
        m_hangarm.retract();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hangarm.stop();
    }
}
