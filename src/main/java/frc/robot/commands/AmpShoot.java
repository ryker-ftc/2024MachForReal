package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class AmpShoot extends CommandBase {
	Conveyor m_Conveyor;

    public AmpShoot(Conveyor conveyor) {
        m_Conveyor = conveyor;
        addRequirements(m_Conveyor);
    }

    @Override
    public void execute() {
        m_Conveyor.ampShoot();
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
