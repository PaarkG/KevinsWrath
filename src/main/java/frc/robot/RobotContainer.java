// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterBeamBreak;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private CommandXboxController driveController;
  private DriveSubsystem swerve;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private IndexSubsystem indexer;
  private PivotSubsystem pivot;
  private ShooterBeamBreak beamBreak;

  public RobotContainer() {
    driveController = new CommandXboxController(0);
    swerve = new DriveSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    indexer = new IndexSubsystem();
    pivot = new PivotSubsystem();
    beamBreak = new ShooterBeamBreak();
    configureBindings();
  }

  private void configureBindings() {
    // Drive
    swerve.setDefaultCommand(swerve.teleopDrive(
      driveController::getLeftY, 
      driveController::getLeftX, 
      driveController::getRightX)
      );

    driveController.povUp()
    .whileTrue(swerve.pidToPose(new Pose2d(0, 0, Rotation2d.fromRadians(0))));

    // Intake
    driveController.rightTrigger()
    .whileTrue(pivot.setIntakeState().andThen(indexer.setIntakeState().alongWith(intake.setIntakeState()))
    .until(beamBreak.isBroken()).andThen(indexer.setIdleState().alongWith(intake.setIdleState())));
    
    driveController.rightBumper()
    .onTrue(pivot.setIntakeState().andThen(indexer.setOuttakeState().alongWith(intake.setOuttakeState())))
    .onFalse(indexer.setIdleState().alongWith(intake.setIdleState()));

    // Amp
    driveController.a()
    .onTrue(pivot.setAmpState().andThen(indexer.setAmpState()))
    .onFalse(pivot.setIntakeState().alongWith(indexer.setIdleState()));

    // Shoot
    //SUBWOOFER
    driveController.x()
    .whileTrue(shooter.windUpDefault().alongWith(pivot.setSubwooferState())
    .andThen(indexer.setFeedState().alongWith(shooter.setDefaultShootVelocity()))
    .until(beamBreak.isntBroken()));

    //PODIUM
    driveController.y()
    .whileTrue(shooter.windUpDefault().alongWith(pivot.setPodiumState())
    .andThen(indexer.setFeedState().alongWith(shooter.setDefaultShootVelocity()))
    .until(beamBreak.isntBroken()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
