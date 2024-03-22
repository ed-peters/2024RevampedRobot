// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.beak.BeakCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.beak.FlywheelParams;
import frc.robot.subsystems.beak.FlywheelSubsystem;
import frc.robot.subsystems.beak.PhotoSensor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static final FlywheelParams INTAKE_PARAMS = new FlywheelParams(
          "intake",
          0.0098,
          0.0005,
          12.0 / 56.0,
          (2.5 / 12.0) * Math.PI);

  public static final FlywheelParams SHOOTER_PARAMS = new FlywheelParams(
          "shooter",
          0.0001,
          0.00399,
          16.0 / 25.0,
          (2.78 / 12.0) * Math.PI);

  public static final DigitalInput photoSwitch = new DigitalInput(1);

  public final CommandXboxController controller;
  public final FlywheelSubsystem intake;
  public final FlywheelSubsystem shooter;
  public final PhotoSensor photoSensor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intake = new FlywheelSubsystem(INTAKE_PARAMS);
    shooter = new FlywheelSubsystem(SHOOTER_PARAMS);
    photoSensor = new PhotoSensor();
    controller = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(BeakCommands.shoot(intake, shooter));
    controller.b().onTrue(BeakCommands.eject(intake, shooter));
    controller.x().onTrue(BeakCommands.intake(intake, shooter, photoSensor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("no autonomous routine");
  }
}
