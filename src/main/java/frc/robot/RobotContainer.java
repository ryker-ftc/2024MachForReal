// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton fastSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton slowSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_intakeIn = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton m_intakeOut = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton m_push = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton m_pull = new JoystickButton(driver, XboxController.Button.kB.value);
  

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intaker s_Intaker = new Intaker();
  private final Lifter s_Lifter = new Lifter();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
            // The robotCentric button, when held down, enables axis behavior relative to the field (and requires a working gyroscope).  The default
            // is for movements to apply relative to the robot.
            () -> robotCentric.getAsBoolean(),
            // slowSpeed button, when held, causes translation and rotation to be performed at a slower speed
            () -> slowSpeed.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
    s_Intaker.setDefaultCommand(new RunCommand(s_Intaker::stop, s_Intaker));

    s_Lifter.setDefaultCommand(new RunCommand(s_Lifter::stop, s_Lifter));

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
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

 
    m_intakeIn.whileTrue(new RunCommand(() -> s_Intaker.suck()));
    m_intakeOut.whileTrue(new RunCommand(() -> s_Intaker.blow()));

    m_pull.whileTrue(new RunCommand(() -> s_Lifter.pull()));
    m_push.whileTrue(new RunCommand(() -> s_Lifter.push()));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }

  public void resetToAbsoluteNorth() {
    s_Swerve.resetToAbsoluteNorth();
  }
}
