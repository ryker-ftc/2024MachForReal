package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class SpinUp extends CommandBase {
    private Conveyor m_Conveyor;
    private double speed;

    public SpinUp(Conveyor conveyor, double speed){
        m_Conveyor = conveyor;
        this.speed = speed;
        addRequirements(m_Conveyor);
    }

    @Override
    public void execute(){
        m_Conveyor.sipnUp(speed);

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted){
        m_Conveyor.stop();
    }
}
