package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTemperatureCelcius = new double[] {};

    public double steerAbsolutePositionRad = 0.0;
    public double steerPositionRad = 0.0;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double[] steerCurrentAmps = new double[] {};
    public double[] steerTemperatureCelcius = new double[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {
  }

  public default void setDriveVoltage(double volts) {
  }

  public default void setSteerVoltage(double volts) {
  }

  public default void setDriveBrakeMode(boolean enable) {
  }

  public default void setSteerBrakeMode(boolean enable) {
  }

}
