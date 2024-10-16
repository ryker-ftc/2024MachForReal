// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */

  private final Joystick driver = new Joystick(0);
  private final Joystick driver2 = new Joystick(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final POVButton dPad_Right = new POVButton(driver2, 90, 0);
  private final POVButton dPad_Top = new POVButton(driver2, 0, 0);
  private final POVButton dPad_Left = new POVButton(driver2, 270, 0);
  private final POVButton dPad_Down = new POVButton(driver2, 180);
  private final JoystickButton aButton = new JoystickButton(driver2, XboxController.Button.kA.value);
  private final JoystickButton leftBumper = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
  private final JoystickButton limeLightDriveButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton rightTrigger = new JoystickButton(driver2, 3);
  private final JoystickButton hangarmUpButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton hangarmDownButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton x2Button = new JoystickButton(driver2, XboxController.Button.kX.value);

  /* Driver Buttons */
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);

  private final SendableChooser<String> chooserColor;
  private final SendableChooser<String> chooserTarget;

  private final JoystickButton slowSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton turbo = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final POVButton hangarmLeftDown = new POVButton(driver, 270, 0);
  private final POVButton hangarmRightDown = new POVButton(driver, 90, 0);

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Camera s_Camera = new Camera();
  public final Conveyor s_Conveyor = new Conveyor();
  // public final Hangarm s_Hangarm = new Hangarm();

  // /* Commands */
  public final GroundIntake c_GroundIntake = new GroundIntake(s_Conveyor);
  public final GroundOuttake c_GroundOuttake = new GroundOuttake(s_Conveyor);
  public final UpperIntake c_UpperIntake = new UpperIntake(s_Conveyor);
  public final LimelightDrive c_LimelightDrive = new LimelightDrive(s_Camera, s_Swerve, 30, 50, 0, 0);
  public final LimelightDrive c_runTheTrap = new LimelightDrive(s_Camera, s_Swerve, 30, 22, 0, 0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    chooserColor = new SendableChooser<String>();
    chooserColor.setDefaultOption("Red alliance", "red");
    chooserColor.addOption("Blue alliance", "blue");

    chooserTarget = new SendableChooser<String>();
    chooserTarget.setDefaultOption("Straight forward", "drive forward");
    chooserTarget.setDefaultOption("Boom and Zoom", "boom and zoom");
    chooserTarget.addOption("4 note auto", "4 note auto"); // etc
    chooserTarget.addOption("2 note auto", "2 note auto");
    chooserTarget.addOption("Nothing", "none");

    SmartDashboard.putData("Color Auto Selector", chooserColor);
    SmartDashboard.putData("Target Auto Selector", chooserTarget);
    // SendableRegistry.setName(chooser, "Auto Selector");

    // new ShuffleboardWrapper(chooser);

    // NamedCommands.registerCommand("shoot", new Shoot(s_Conveyor, 1).withTimeout(2));
    // NamedCommands.registerCommand("intake", new InstantCommand(() -> s_Conveyor.groundIntake()));
    // NamedCommands.registerCommand("stop", new InstantCommand(() -> s_Conveyor.stop()));

  }

  public void teleopInit() {
    this.resetToAbsoluteNorth();

    s_Swerve.setDefaultCommand(
        // Command that's continuously run to update the swerve state
        new TeleopSwerve(
            // The Swerve subsystem
            s_Swerve,
            // getRawAxis() returns a value for the controller axis from -1 to 1
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            // robotCentric and slowSpeed are both buttons on the joystick
            // The robotCentric button, when held down, enables axis behavior relative to
            // the field (and requires a working gyroscope). The default
            // is for movements to apply relative to the robot.
            () -> robotCentric.getAsBoolean(),
            // slowSpeed button, when held, causes translation and rotation to be performed
            // at a slower speed
            () -> slowSpeed.getAsBoolean(),
            () -> turbo.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    aButton.whileTrue(c_GroundIntake);
    leftBumper.whileTrue(c_GroundOuttake);
    limeLightDriveButton.whileTrue(c_LimelightDrive);

    xButton.whileTrue(new RepeatCommand(new InstantCommand(() -> s_Swerve.setX())));

    // x2Button.whileTrue(new GroundIntakeColor(s_Conveyor));

    dPad_Top.whileTrue(new SpinUp(s_Conveyor, 1).withTimeout(0.5).andThen(new Shoot(s_Conveyor, 1)));
    dPad_Left.whileTrue(new SpinUp(s_Conveyor, 0.5).withTimeout(0.5).andThen(new Shoot(s_Conveyor, 0.5)));

    // dPad_Top.whileTrue(new SpinUp(s_Conveyor, 1));
    dPad_Down.whileTrue(new Shoot(s_Conveyor, 1));

    // // no no delete ryker
    // hangarmDownButton.whileTrue(new HangarmDown(s_Hangarm));
    // hangarmUpButton.whileTrue(new HangarmUp(s_Hangarm));

    // hangarmLeftDown.whileTrue(new InstantCommand(() -> s_Hangarm.setLeftMotor(0.2))/*.withTimeout(0.05)
    // .andThen(new InstantCommand(() -> s_Hangarm.stop()))*/);
    // hangarmRightDown.whileTrue(new InstantCommand(() -> s_Hangarm.setRightMotor(-0.2))/*.withTimeout(0.05)
    // .andThen(new InstantCommand(() -> s_Hangarm.stop()))*/);

  }

  public void configureTestButtons() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {

    // return new BusterAuto(this, chooserColor, chooserTarget, s_Camera);
  // }

  public void resetToAbsoluteNorth() {
    s_Swerve.resetToAbsoluteNorth();
  }

  /**
   * This is run constantly as soon as the robot is plugged in.
   */
  public void periodic() {

    SmartDashboard.putString("Choosen Auto Color", chooserColor.getSelected());
    SmartDashboard.putString("Choosen Auto Target", chooserTarget.getSelected());

  }

  public void killTeleop() {
    s_Swerve.removeDefaultCommand();
  }

}
