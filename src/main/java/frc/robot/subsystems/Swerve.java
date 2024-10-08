//ryker
package frc.robot.subsystems;

import java.util.Date;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private PigeonIMU gyro = new PigeonIMU(35); /* example Pigeon with device ID 0 */
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  // private SwerveDriveKinematics kinematics = new SwerveDriveKinematics();

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  public Swerve() {
    // gyro.configFactoryDefault();
    zeroGyro(0);

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveFromChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(15, 0.0, 0), // Translation PID constants
        new PIDConstants(15, 0.0, 0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        0.41, // Drive base radius in meters. Distance from robot center to furthest module. FIX THIS!!!!!!!!
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public void drive(

      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SmartDashboard.putNumber("wheel 1 speede", translation.getX());
    SmartDashboard.putNumber("drive() translation X", translation.getX());
    SmartDashboard.putNumber("drive() translation Y", translation.getY());
    SmartDashboard.putNumber("drive() rotation", rotation);
    SmartDashboard.putNumber("drive() fieldRelative", fieldRelative ? 1.0 : 0.0);
    // Each SwerveModuleState contains an angle and a speedMetersPerSecond for the
    // module.
    // Calculate new values for these based on the values from the joystick. The
    // values defined in
    // the Constants for this are critical.
    //ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    SmartDashboard.putBoolean("is Field Relative?", fieldRelative);
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), translation.getY(), rotation, getYaw());
    ChassisSpeeds robotCentricSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    ChassisSpeeds speeds = fieldRelative ? fieldRelativeSpeeds : robotCentricSpeeds;

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        // If the fieldRelative/robotCentric button is pressed, calculate individual
        // module
        // angles and speeds from an absolute/field point of view.
        // Otherwise, calculate them relative to the robot.
        // The calculated ChassisSpeeds object contains the intended x and y velocities
        // of
        // the robot (in m/s) as well as the intended angular velocity of the robot.
        speeds);
        /*fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));*/
    // This function ensures that no individual Swerve module wheel is driven faster
    // than it
    // can physically handle. If any of the individual module speeds are above the
    // defined
    // constant, all speeds are readjusted accordingly.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    lastChassisSpeeds = speeds;

    // Updated each module with our desired speed and angle for it
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, 0);
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);


    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    lastChassisSpeeds = chassisSpeeds;


    // Updated each module with our desired speed and angle for it
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
    }
  }

  // note: pose and odometry are only used for status readouts
  public Pose2d getPose() {

    SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("gyro angle", getYaw().getDegrees());
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
      SmartDashboard.putNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro(double angle) {
    gyro.setYaw(angle);
    gyro.setFusedHeading(angle);

  }

  public Rotation2d getYaw() {
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    gyro.getGeneralStatus(genStatus);
    double [] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(ypr[0]);
    // return (Constants.Swerve.invertGyro)
    //     ? Rotation2d.fromDegrees(360 - gyro.getAngle())
    //     : Rotation2xd.fromDegrees(gyro.getAngle());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    publisher.set(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderAbsolutePosition());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getInternalAngle());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    // SmartDashboard.putNumber("Pose X", getPose().getX());
  }

  public void resetToAbsoluteNorth() {
    //zeroGyro();
    for (SwerveModule mod : mSwerveMods) {
  
      mod.resetToAbsoluteNorth();
    }
  }

  public void setX() {
     //removeDefaultCommand();
     SmartDashboard.putString("Last X?", new Date().toString());
     mSwerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-135)), false, false); //Front Left
     mSwerveMods[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)), false, false); //Front Right
     mSwerveMods[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)), false, false); //Back Left
     mSwerveMods[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-135)), false, false); //Back Right
  }


  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = getStates();
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]);
  }
}