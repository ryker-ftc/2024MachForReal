package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToAngleCommand;
// import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Camera;
import frc.robot.commands.LimelightDrive;
import frc.robot.commands.Shoot;



public class BusterAuto extends SequentialCommandGroup {
  private RobotContainer m_robotContainer;
  private LimelightDrive limelightDrive;
  private Shoot c_shoot;
  private Trajectory trajectory;
  private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public BusterAuto(RobotContainer container, SendableChooser<String> chooser, Camera camera) {
    m_robotContainer = container;
    limelightDrive = new LimelightDrive(camera, m_robotContainer.s_Swerve, 10);
    c_shoot = new Shoot(m_robotContainer.s_Conveyor, 1);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Swerve.swerveKinematics);


    switch (chooser.getSelected()) {
      case "speaker blue":
        trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
            new Translation2d(0.5, 0)
          ),
          new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
          trajectoryConfig);


        addCommands(
          new InstantCommand(() -> c_shoot.execute()),
          new WaitCommand(1),
          trajectoryCmd(trajectory)

          
        
         // new WaitCommand(1),

          
        );

      case "amp blue":

      case "speaker red":

      case "amp red":
  

      // case "speaker blue":
      //   addCommands(
      //   new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2,0), 0, false, false)),
      //     new WaitCommand(2),
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));
      //   addCommands(
      //     limelightDrive,
      //     new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(0.6)));
      //   break;
      // case "speaker red":
      //   addCommands(
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2,0), 0, false, false)),
      //     new WaitCommand(2),
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));
      //   addCommands(
      //   limelightDrive,
      //    new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(0.6)));
      //   break;
      // case "amp blue":
      //   addCommands(
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2,0), 0, false, false)),
      //     new WaitCommand(2),
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));

      //   addCommands(
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0,0),2,false,false)),
      //     new WaitCommand(2),
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0,0),0,false,false)),
      //     new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)));
      //   break;
      // case "amp red":
      //   addCommands(
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2,0), 0, false, false)),
      //     new WaitCommand(2),
      //     new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));
      //   addCommands(
      //     new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
      //     new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(0.6)));
      //   break;
    
      // case "straight":
      //   addCommands(
      //       //  new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
      //       //  new WaitCommand(3),
      //       //  new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2, 0), 0, false, false)),
      //       new WaitCommand(2),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));
      //   break;
      // case "left":
      //   addCommands(
      //     // new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
      //     //  new WaitCommand(3),
      //     //  new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.25, 0), 0, false, false)),
      //       new WaitCommand(2.5),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false))

      //   );
      //   break;
      // case "right":
      //   addCommands(
      //       // new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
      //       // new WaitCommand(3),
      //       // new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, -4.5), 0, false, true)),
      //       new WaitCommand(3),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(4.5, 0), 0, false, true)),
      //       new WaitCommand(3),
      //       new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))

      //   );
      //   break;
      case "none":
        break;

    }
  }

  public SwerveControllerCommand trajectoryCmd(Trajectory trajectory) {
    return new SwerveControllerCommand(
      trajectory,
      m_robotContainer.s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      xController,
      yController,
      thetaController,
      m_robotContainer.s_Swerve::setModuleStates,
      m_robotContainer.s_Swerve
    );
  }

}
