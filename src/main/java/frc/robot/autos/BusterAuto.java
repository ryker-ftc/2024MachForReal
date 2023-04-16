package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BusterAuto extends SequentialCommandGroup {
  private RobotContainer m_robotContainer;
  public BusterAuto(RobotContainer container, SendableChooser<String> chooser) {
    m_robotContainer = container;
   
    addCommands(
        // Lift the lifter 4 rotations
        // new InstantCommand(() -> m_robotContainer.s_Lifter.setToPosition(4)),
        // new WaitCommand(5),
        // Push out the cube using the intaker
        // new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
        // new WaitCommand(2),
        // new InstantCommand(() -> m_robotContainer.s_Intaker.stop()),

        // Move backward
        new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(1, 0), 0, false, true)),
        new WaitCommand(3),

        new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))
    // Stop the robot

    );

  }

}
