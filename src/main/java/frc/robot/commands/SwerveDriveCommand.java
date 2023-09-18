// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.Swerve;

public class SwerveDriveCommand extends CommandBase {

  private final Swerve swerve;
  private final DoubleSupplier forward;
  private final DoubleSupplier strafe;
  private final DoubleSupplier rotation;
  private final Boolean fieldOriented;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(Swerve swerve, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation,
      Boolean fieldOriented) {
    this.swerve = swerve;
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;
    this.fieldOriented = fieldOriented;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("SwerveDriveCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble(), fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    System.out.println("SwerveDriveCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
