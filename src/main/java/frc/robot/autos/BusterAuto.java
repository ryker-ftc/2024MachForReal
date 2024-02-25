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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToAngleCommand;
// import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Camera;
import frc.robot.commands.LimelightDrive;
import frc.robot.commands.Shoot;



public class BusterAuto extends SequentialCommandGroup {
  private int colorFactor = 1;
  private RobotContainer m_robotContainer;
  private LimelightDrive limelightDrive;
  private Shoot c_shoot;
  private Trajectory trajectory;
  private TrajectoryConfig trajectoryConfig;
  private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);


  public BusterAuto(RobotContainer container, SendableChooser<String> chooserColor, SendableChooser<String> chooserTarget, Camera camera) {
    m_robotContainer = container;
    limelightDrive = new LimelightDrive(camera, m_robotContainer.s_Swerve, 10);
    c_shoot = new Shoot(m_robotContainer.s_Conveyor, 1);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Swerve.swerveKinematics);


    if (chooserColor.getSelected() == "blue")
      colorFactor = -1;
      


    switch (chooserTarget.getSelected()) {
      case "speaker":
        


        addCommands(
          // new InstantCommand(() -> c_shoot.execute()),
          // new WaitCommand(1),
          
          new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          trajectoryCmd(new double[]{}, new double[]{}, 1, 0),
          // check if have to reset odometry


          
          new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          new WaitCommand(3),
          new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(0)),
          trajectoryCmd(new double[]{}, new double[]{}, 1, 2),
          new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false))

          
        
         // new WaitCommand(1),

          
        );

      case "none":
        break;

    }
  }

  public SwerveControllerCommand trajectoryCmd(double[] waypointX, double[] waypointY, double endX, double endY) {
    List<Translation2d> waypointList = new ArrayList<Translation2d>(0);
    for (int i = 0; i < waypointX.length; i++)
      waypointList.add(new Translation2d(waypointX[i], waypointY[i] * colorFactor));

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0 * colorFactor, new Rotation2d(0)),
      waypointList,
      // List.of(
      //   new Translation2d(0.1, 0 * colorFactor)
      // ),
      new Pose2d(endX, endY * colorFactor, Rotation2d.fromDegrees(0)),
      trajectoryConfig);
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