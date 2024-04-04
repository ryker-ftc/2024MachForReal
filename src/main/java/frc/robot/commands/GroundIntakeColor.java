package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;

public class GroundIntakeColor extends Command {
	Conveyor m_Conveyor;
    double offset = 0;

    public GroundIntakeColor(Conveyor conveyor) {
        m_Conveyor = conveyor;
        addRequirements(m_Conveyor);
    }

    @Override
    public void execute() {
        offset = 170 - m_Conveyor.colorSensor.readColors().getR();
        SmartDashboard.putNumber("COLORSENSOR OFFSET * 0.05", offset * 0.05);
        m_Conveyor.setIntakeMotorBottom(MathUtil.clamp(offset * -0.05, -1, 1));
        m_Conveyor.setIntakeMotorTop(MathUtil.clamp(offset * 0.05, 1, 1));
        m_Conveyor.setPlacementMotor(MathUtil.clamp(offset * 0.03, -0.6, 0.6));

    }

    @Override
    public boolean isFinished() {
        return Math.abs(offset) < 3;
    }

    @Override
    public void end(boolean interrupted) {
        m_Conveyor.stop();
    }
}
