package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class Shoot extends CommandBase {
	private Conveyor m_Conveyor;
    private double speed;
    private boolean hasRun = false;

    public Shoot(Conveyor conveyor, double speed) {
        m_Conveyor = conveyor;
        this.speed = speed;
        addRequirements(m_Conveyor);
    }

    @Override
    public void execute() {
        m_Conveyor.shoot(speed);
        hasRun = true;
    }

    @Override
    public boolean isFinished() {
        return hasRun;
    }

    @Override
    public void end(boolean interrupted) {
        m_Conveyor.stop();
    }
}
