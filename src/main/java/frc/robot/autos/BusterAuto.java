// package frc.robot.autos;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.AutoConstants;

// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import frc.robot.commands.LimelightDrive;
// import frc.robot.commands.Shoot;
// import frc.robot.subsystems.*;
// import frc.robot.commands.*;;

// public class BusterAuto extends SequentialCommandGroup {
//   private int colorFactor = 1;
//   private RobotContainer m_robotContainer;
//   private Swerve swerve;
//   private LimelightDrive limelightDrive;
//   private Conveyor conveyor;
//   private Trajectory trajectory;
//   private TrajectoryConfig trajectoryConfig;
//   private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
//   private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
//   private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
//       AutoConstants.kThetaControllerConstraints);

//   public BusterAuto(RobotContainer container, SendableChooser<String> chooserColor,
//       SendableChooser<String> chooserTarget, Camera camera) {
//     m_robotContainer = container;
//     swerve = m_robotContainer.s_Swerve;
//     conveyor = m_robotContainer.s_Conveyor;
    
//     //limelightDrive = new LimelightDrive(camera, m_robotContainer.s_Swerve, 10, );

//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     trajectoryConfig = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         .setKinematics(Constants.Swerve.swerveKinematics);

//     if (chooserColor.getSelected() == "blue")
//       colorFactor = -1;

//     switch (chooserTarget.getSelected()) {

//       case "2 note auto":

//         addCommands(

//             new InstantCommand(() -> swerve.zeroGyro(0)),
//             new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 2, 60, 0, 0),
//             new SpinUp(conveyor, 1).withTimeout(0.5).andThen(new Shoot(conveyor, 1).withTimeout(2)),

//             new ParallelCommandGroup(
//                 new GroundIntakeColor(conveyor).withTimeout(6),
//                 new SequentialCommandGroup(
//                     new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 2, 110, 0, 0),
//                     new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 2, 55, 0, 0))),
//             new SpinUp(conveyor, 1).withTimeout(0.5).andThen(new Shoot(conveyor, 1).withTimeout(2))

//         );

//         break;

//       case "4 note auto":

//         addCommands(
//             new InstantCommand(() -> swerve.zeroGyro(0)),
//             new SpinUp(conveyor, 1).withTimeout(0.5).andThen(new Shoot(conveyor, 1).withTimeout(2)),

//             new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
//             new ParallelCommandGroup(
//                 new GroundIntakeColor(conveyor).withTimeout(6),
//                 new SequentialCommandGroup(
//                     AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 1")),
//                     new WaitCommand(0.3),

//                     new LimelightDrive(m_robotContainer.s_Camera, m_robotContainer.s_Swerve, 5, 55, 0, 0)

//                 )),
//             new SpinUp(conveyor, 1).withTimeout(0.5).andThen(new Shoot(conveyor, 1).withTimeout(2)),

//             new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
//             new ParallelCommandGroup(
//                 new GroundIntakeColor(conveyor),
//                 new SequentialCommandGroup(
//                     AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 2")),
//                     new WaitCommand(0.2),
//                     new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.6, 5.55, swerve.getYaw()))),
//                     AutoBuilder.followPath(PathPlannerPath.fromPathFile("2 to speaker")))),
//             new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.5, 5.55, swerve.getYaw()))),
//             new ParallelCommandGroup(
//                 new GroundIntakeColor(conveyor),
//                 new SequentialCommandGroup(
//                     AutoBuilder.followPath(PathPlannerPath.fromPathFile("speaker to 3")),
//                     new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.6, 4.1, swerve.getYaw()))),
//                     AutoBuilder.followPath(PathPlannerPath.fromPathFile("3 to speaker"))))

//         );

//         break;

//       case "drive forward":
//         addCommands(
//             new InstantCommand(() -> swerve.zeroGyro(0)),
//             new WaitCommand(12),
//             new InstantCommand(
//                 () -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.2, 0), 0, false, true)),
//             new WaitCommand(2),
//             new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)));
//         break;

//       case "none":
//         addCommands(
//           new InstantCommand(() -> swerve.zeroGyro(0))
//         );
//         break;
//       case "boom and zoom":
//         addCommands(
//             new InstantCommand(() -> swerve.zeroGyro(45 * colorFactor)),
//             new SpinUp(conveyor, 1).withTimeout(0.5).andThen(new Shoot(conveyor, 1).withTimeout(2)),
//             new WaitCommand(1),
//             new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
//             new WaitCommand(7.5),
//             new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.1, 0), 0, false, true)),
//             new WaitCommand(1.5),
//             new InstantCommand(
//                 () -> m_robotContainer.s_Swerve.drive(new Translation2d(-0.1, 0.1 * colorFactor), 0, false, true)),
//             new WaitCommand(1.5),
//             new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))
//         );
//     }
//   }

//   public SwerveControllerCommand trajectoryCmd(double[] waypointX, double[] waypointY, double endX, double endY) {
//     List<Translation2d> waypointList = new ArrayList<Translation2d>(0);
//     for (int i = 0; i < waypointX.length; i++)
//       waypointList.add(new Translation2d(waypointX[i], waypointY[i] * colorFactor));

//     trajectory = TrajectoryGenerator.generateTrajectory(
//         new Pose2d(0, 0 * colorFactor, new Rotation2d(0)),
//         waypointList,

//         new Pose2d(endX, endY * colorFactor, Rotation2d.fromDegrees(0)),
//         trajectoryConfig);
//     return new SwerveControllerCommand(
//         trajectory,
//         m_robotContainer.s_Swerve::getPose,
//         Constants.Swerve.swerveKinematics,
//         xController,
//         yController,
//         thetaController,
//         m_robotContainer.s_Swerve::setModuleStates,
//         m_robotContainer.s_Swerve);
//   }

//   public void intakeShootLoop(double[] waypointX, double[] waypointY, double endX, double endY) {
//     addCommands(
//         new InstantCommand(() -> m_robotContainer.s_Conveyor.groundIntake()),
//         trajectoryCmd(waypointX, waypointY, endX, endY),
//         new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()),
//         m_robotContainer.c_LimelightDrive,
//         new InstantCommand(() -> m_robotContainer.s_Conveyor.shoot(1)),
//         new WaitCommand(2),
//         new InstantCommand(() -> m_robotContainer.s_Conveyor.stop()));

//   }

// }