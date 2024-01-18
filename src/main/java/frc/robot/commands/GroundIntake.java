package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class GroundIntake extends CommandBase {
	Conveyor m_Conveyor;

    public GroundIntake(Conveyor conveyor) {
        m_Conveyor = conveyor;
        addRequirements(m_Conveyor);
    }

    @Override
    public void execute() {
        m_Conveyor.groundIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Conveyor.stop();
    }
}
