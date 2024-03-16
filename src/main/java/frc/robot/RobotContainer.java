// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = 2.5; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1. * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final XboxController xbox2 = new XboxController(1);
  private final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SendableChooser<Command> autoChooser;
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // triggers
  public Trigger driverY = new Trigger(()->xbox2.getYButton());
  public Trigger padUp = new Trigger(()->(xbox2.getPOV()==0));
  public Trigger padDown = new Trigger(()->(xbox2.getPOV()==180));

  private void configureBindings() {
    
    
    drivetrain.setDefaultCommand(
      new SwerveDriveControl(
        drivetrain, 
        () -> -joystick.getLeftX(),  //Translation 
        () -> -joystick.getLeftY(),  //Translation
        () -> -joystick.getRightX(), //Rotation
        joystick.povUp(), 
        joystick.povDown(), 
        joystick.y(), //Face Forward
        joystick.b(), //Face Right
        joystick.a(), //Face Backwards
        joystick.x()  //Face Left
      )
    );

    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.setFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void registerNamedCommands() {

  }

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();

    registerNamedCommands();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
