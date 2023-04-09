package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

// CB: This class represents one of the four hardware swerve modules
public class SwerveModule {
  public int moduleNumber;
  private double lastAngle;
  private double angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  // This class represents the CANcoder for the swerve module
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    // /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    // angleEncoder.setPosition(angleOffset);

  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    // angleController.setFeedbackDevice(integratedAngleEncoder);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();
    Timer.delay(1);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void resetToAbsolute() {
    angleEncoder.configMagnetOffset(angleOffset);
    double canCoderDegrees = getCanCoder().getDegrees();
    integratedAngleEncoder.setPosition(canCoderDegrees);
  }

  public void resetToAbsoluteNorth() {
    // CANcoder absolute position in degrees
    double canCoderDegrees = getCanCoder().getDegrees();
    double absolutePosition = canCoderDegrees;
    SmartDashboard.putNumber("M1- CanDegrees: " + moduleNumber, canCoderDegrees);
    SmartDashboard.putNumber("M1- Integrated Angle Motor Position: " + moduleNumber, integratedAngleEncoder.getPosition());
    // TODO: This has to be zero after we reset the cancoder in CTREConfig.java 
    angleController.setReference(0, ControlType.kPosition);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    //Prevent rotating module if speed is less then 1%. Prevents jittering.
   double angle = (Math.abs(desiredState.speedMetersPerSecond) <=
    (Constants.Swerve.maxSpeed * 0.01))
    ? lastAngle
    : desiredState.angle.getDegrees();

    SmartDashboard.putNumber("Angle: " + moduleNumber, angle);
    SmartDashboard.putNumber("Last Angle: " + moduleNumber,
    lastAngle);
    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    // This optimize() instruction is taken straight from the ChiefDelphi example, I can find
    // no documentation on what it actually does.  Could try commenting it out if problems
    // persist.
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  // Get angular measurement from the relative encoder integrated with the rotation motor
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  // CB: Returns an angular measurement from the CANcoder, as the class Rotation2D
  public Rotation2d getCanCoder() {
    // CB: angleEncoder is the CANcoder for this module.  getAbsolutePosition() returns the angular
    // position in degrees; and Rotation2d.fromDegrees converts it into a Rotation2d
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  // The returned state consists of the current velocity of the wheel and the angle of the swerve module
  // but calculated from the rotation motor's integrated encoder rather than the CANcoder
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    SmartDashboard.putNumber("angleEncoder position " + moduleNumber, angleEncoder.getPosition());
    SmartDashboard.putNumber("angleOffset degrees " + moduleNumber, angleOffset);

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromDegrees(angleEncoder.getPosition() - angleOffset));
  }
}
