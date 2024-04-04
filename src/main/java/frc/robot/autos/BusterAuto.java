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
// import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Camera;
import frc.robot.commands.LimelightDrive;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Conveyor;

public class BusterAuto extends SequentialCommandGroup {
  private int colorFactor = 1;
  private RobotContainer m_robotContainer;
  private Swerve swerve;
  private LimelightDrive limelightDrive;
  private Conveyor Conveyor;
  private Trajectory trajectory;
  private TrajectoryConfig trajectoryConfig;
  private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);


  public BusterAuto(RobotContainer container, SendableChooser<String> chooserColor, SendableChooser<String> chooserTarget, Camera camera) {
    m_robotContainer = container;
    swerve = m_robotContainer.s_Swerve;
    Conveyor = m_robotContainer.s_Conveyor;
    
    //limelightDrive = new LimelightDrive(camera, m_robotContainer.s_Swerve, 10, );
<<<<<<< HEAD
    Conveyor = container.s_Conveyor;
=======
>>>>>>> 9ddfc22d46c5a1d6594ce84504522ddce25ab759

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Swerve.swerveKinematics);


    if (chooserColor.getSelected() == "blue")
      colorFactor = -1;
      


    switch (chooserTarget.getSelected()) {

      case "2 note auto":

        addCommands(
          new PathPlannerAuto("2 note")

          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 2, 45),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          // new WaitCommand(2),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),  



          
        
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 3.5, 98),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.2, 0), 0, false, true)),
          // new WaitCommand(1), 
          // new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)),
          // new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)),          

          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 5, 45),

          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          // new WaitCommand(1),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 5, 80)


        );

        break;


      case "4 note auto":

        // new PathPlannerAuto("auto test");
        
        addCommands(
          new InstantCommand(() -> swerve.zeroGyro(0)),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 1")),
           new InstantCommand(() -> S_Conveyor.shoot()),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.6, 7, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("1 to speaker")),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 2")),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.6, 5.55, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("2 to speaker")),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 3")),
          new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.6, 4.1, swerve.getYaw()))),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("3 to speaker"))
        );


        //   // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
        //   AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 2")),
        //   // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
        //   // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 100),
        //   new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
        //   new WaitCommand(2),
        //   new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())


          // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          // AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 1")),
          // AutoBuilder.followPath(PathPlannerPath.fromPathFile("1 to speaker")),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 100),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          // new WaitCommand(2),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())







          

        
          // new InstantCommand(() -> c_shoot.execute()),
          // new WaitCommand(1),

          // intakeShootLoop(new double[]{1.1}, new double[]{0}, 1.2, 1.45);
          // intakeShootLoop(new double[]{0, -0.6}, new double[]{-1.45, -1.45}, 0, 0);
          // intakeShootLoop(new double[]{0.6}, new double[]{0}, 0, 0);
          // intakeShootLoop(new double[]{0, -0.6}, new double[]{1.45, 1.45}, 0, 0);
          
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          // // trajectoryCmd(new double[]{-1.1}, new double[]{0}, -1.2, 1.45),
          
          // new SwerveControllerCommand(
          //   TrajectoryGenerator.generateTrajectory(
          //   new Pose2d(0, 0 * colorFactor, new Rotation2d(0)),
          //   // waypointList,
          //   List.of(
          //     // new Translation2d(-1.1, 0 * colorFactor)
          //   ),
          //   new Pose2d(0, 1 * colorFactor, Rotation2d.fromDegrees(0)),
          //   trajectoryConfig),
          //   m_robotContainer.s_Swerve::getPose,
          //   Constants.Swerve.swerveKinematics,
          //   xController,
          //   yController,
          //   thetaController,
          //   m_robotContainer.s_Swerve::setModuleStates,
          //   m_robotContainer.s_Swerve
          // ),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 100),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(0.2)),
          // new WaitCommand(2),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())

          // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          // trajectoryCmd(new double[]{0, -0.6}, new double[]{-1.45, -1.45}, 0, 0),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 100),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          // new WaitCommand(2),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),

          // new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
          // trajectoryCmd(new double[]{0, -0.6}, new double[]{-1.45, -1.45}, 0, 0),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
          // new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 100),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
          // new WaitCommand(2),
          // new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())



          // check if have to reset odometry

        // );
          
          

          
        
         // new WaitCommand(1),

          
        

        break;

      case "drive forward":
        addCommands(
          new WaitCommand(12),
           new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.2, -0.2 * colorFactor), 0, false, true)),
           new WaitCommand(2),
           new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))
        );
        break;

      case "none":
        break;
      case "boom and zoom":
      addCommands(
        // new SpinUp(s_Conveyor, 1).withTimeout(0.5).andThen(new Shoot(s_Conveyor, 1).withTimeout(2)),
        new WaitCommand(1),
        new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
        new WaitCommand(10),
        new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.1, 0), 0, false, true)),
        new WaitCommand(1.5),
        new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.1, 0.1*colorFactor), 0, false, true)),
        new WaitCommand(1.5),
        new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))
      );
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

  public void intakeShootLoop(double[] waypointX, double[] waypointY, double endX, double endY) {
    addCommands(
      new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
      trajectoryCmd(waypointX, waypointY, endX, endY),
      new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
      m_robotContainer.c_LimelightDrive,
      new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
      new WaitCommand(2),
      new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())
    );

    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
    //   trajectoryCmd(waypointX, waypointY, endX, endY),
    //   new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
    //   m_robotContainer.c_LimelightDrive,
    //   new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
    //   new WaitCommand(2),
    //   new InstantCommand(() -> m_robotContainer.s_Conveyor.stop())
    // );
  }

}