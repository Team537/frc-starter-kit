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
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

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
    if (Math.abs(forward.getAsDouble()) < 0.01) {
      xSpeed = 0;
    } else {
      xSpeed = forward.getAsDouble();

    }

    if (Math.abs(strafe.getAsDouble()) < 0.01) {
      ySpeed = 0;
    } else {
      ySpeed = strafe.getAsDouble();

    }

    if (Math.abs(rotation.getAsDouble()) < 0.01) {
      rotSpeed = 0;
    } else {
      rotSpeed = rotation.getAsDouble();

    }

    swerve.drive(xSpeed, ySpeed, rotSpeed, fieldOriented);
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
