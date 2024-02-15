package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class LimelightDrive extends CommandBase{ 
    private Camera m_camera;
    private Swerve m_swerve;
    private boolean complete = false;
    private double timeout;
    private Timer timer = new Timer();
    private SlewRateLimiter limiter = new SlewRateLimiter(3.0);
    private final double shootDistance = 65.0;

    public LimelightDrive(Camera camera, Swerve swerve, double timeout) {
        m_camera = camera;
        m_swerve  = swerve;
        this.timeout = timeout;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        complete = false;
    }

    @Override
    public void execute() {
        final double anglekP = 0.025;
        final double distancekP = 0.025;
        double distanceError = shootDistance - m_camera.getDistanceToGoal();
        double angleError = m_camera.getHeading();

        double angularSpeed = MathUtil.clamp(angleError * anglekP, -Constants.Swerve.maxAngularVelocity*0.5, Constants.Swerve.maxAngularVelocity*0.5);
        double linearSpeed = limiter.calculate(distanceError * distancekP);
    
        if (Math.abs(angleError) > 2 && Math.abs(distanceError) > 2 && timer.get() < timeout) {
            m_swerve.drive(new Translation2d(-linearSpeed,0), angularSpeed, false, true);
        } else {
            complete = true;
        }
    }    

    @Override
    public boolean isFinished() {
        return complete;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(0,0), 0, false, true);
        timer.stop();
    }
    
}
