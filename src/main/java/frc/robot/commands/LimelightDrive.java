package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LimelightDrive extends CommandBase{ 
    private Camera m_camera;
    private Swerve m_swerve;
    private TurnToAngleCommand c_turnToAngleCommand;

    public LimelightDrive(Camera camera, Swerve swerve) {
        m_camera = camera;
        m_swerve  = swerve;
        c_turnToAngleCommand = new TurnToAngleCommand(swerve, 0, 0);
    }

    public void execute() {}    

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {

    }
    
}
