package frc.robot;

import javax.swing.ImageIcon;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.056;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(22.75);
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1);// (150.0 / 7.0); //

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation 1 */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
    public static final int numberOfSensorCountsPerRevolution = 42;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second //4.5
    //public static final double defaultSpeed = 4.0;
    public static final double maxAngularVelocity = 6; // 11.5

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 11;
      public static final int angleOffset = -164; //101 //-64 //359 //89
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 9;
      public static final int angleOffset = -153; //14 //-249 //8 //7
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 12;
      public static final int angleOffset = -350; //107 //-348 //0 //-68
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final int angleOffset = -258; //298 //-259 //-359 //273
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class ConveyorConstants {
    /* Conveyor module - Module 4 */
    public static final class Mod4 {
      public static final int intakeMotorChannel = 1;
      public static final int intakeMotorID = 13;
      public static final int intakeEncoderPort = 15;
      public static final int placementMotorID = 14;
      public static final int flywheelMotorID = 15;


      //  public static final boolean kEncoderReversed = false;
      // public static final int kEncoderCPR = 42;
      // public static final double kEnocderDistancePerPulse = 1.0 / (double) kEncoderCPR;

    }
  }

  public static final class LiftConstants {
    /* Lift module - Module 5 */
    public static final class Mod5 {
     /*Extra uneeded stuff (may need if we use the joystick to control the lifter)*/
     public static final double kLiftGearing = 12;
     public static final double kLifterDrumRadius = Units.inchesToMeters(0.046875);
     public static final double kCarriageMass = 4.0;
     public static final double kEncoderDistancePerPulse = (1.0/42) * (kLifterDrumRadius * 2 * Math.PI);
     public static final double kLifterMinHeight = 0;
     public static final double kLifterMaxHeight = Units.inchesToMeters(50.0);
      public static final int kLiftMotorID = 14;

      public static final double kPulleyRatio = ((kLifterDrumRadius * 2.0) * Math.PI);
      public static final int kEncoderChannelA = 14;
      public static final int kEncoderChannelB = 15;

      /* Lifter Motor PID Values */
      public static final double angleKP = 0.01;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;
      public static final double angleKFF = 0.0;

      

      public static final int demand = 1;

      /*Lifter Motor Conversion Factors */
      public static final double kLifterMotorPositionFactor = 1;
      public static final double kLifterMotorVelocityFactor = 1;

      public static final DigitalInput toplimitSwitch = new DigitalInput(0);
      public static final DigitalInput bottomlimitSwitch = new DigitalInput(1);
      // PWMVictorSPX motor = new PWMVictorSPX(0);
      // Joystick joystick = new Joystick(0);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4; // 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
