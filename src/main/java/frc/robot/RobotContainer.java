// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
// import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.drivetrain.LineUpWithConePath;
import frc.robot.commands.limelight.LineUpWithCone;
import frc.robot.commands.limelight.LineUpWithCube;
import frc.robot.commands.limelight.ScanCone;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI * 4; // Half a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Limelight limelight = new Limelight();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentricFacingAngle turn = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1)
      .withTargetDirection(new Rotation2d(3))
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> turn
    //         .withVelocityX(-joystick.getLeftY() * MaxSpeed)
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed)
    //     ));

    joystick.rightTrigger(0.1).whileTrue(new LineUpWithCone(drivetrain, limelight));

    joystick.leftTrigger(0.1).whileTrue(new LineUpWithCube(drivetrain, limelight));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    NamedCommands.registerCommand("ConeLineUp", new LineUpWithCone(drivetrain, limelight));
    NamedCommands.registerCommand("LineUpConePath", new LineUpWithConePath(drivetrain, limelight, "Test Auto", 0));
    NamedCommands.registerCommand("ScanCone", new ScanCone(limelight));

    configureBindings();
  }

  public boolean isAllianceBlue() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return true;
    }
    return false;
  }

  public boolean isAllianceRed() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return true;
    }
    return false;
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return drivetrain.getAutoPath("Test Auto");
  }
}
