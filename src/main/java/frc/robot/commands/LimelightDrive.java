package frc.robot.commands;

import frc.robot.subsystems.*;

public class LimelightDrive {
    private Camera m_camera;
    private Swerve m_swerve;
    private TurnToAngleCommand c_turnToAngleCommand;

    public LimelightDrive(Camera camera, Swerve swerve) {
        m_camera = camera;
        m_swerve  = swerve;
        c_turnToAngleCommand = new TurnToAngleCommand(swerve, 0, 0);
    }
    
}
