package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.YAMLDataHolder;

import frc.robot.utils.LoggedTunableValue;
import frc.robot.utils.ModulePosition;

public class Module extends SubsystemBase {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModulePosition position;
  private Pose2d pose = new Pose2d();

  // public static final double TURNING_MOTOR_GEAR_RATIO = 15.428;
  // public static final int ENCODER_RESOLUTION = 2048;
  // public static final double TURN_ENCODER_METERS_PER_PULSE =

  // 360 / (ENCODER_RESOLUTION * TURNING_MOTOR_GEAR_RATIO);

  private LoggedTunableValue STEER_MOTOR_GEAR_RATIO = new LoggedTunableValue("STEER_MOTOR_GEAR_RATIO");
  private LoggedTunableValue STEER_ENCODER_METERS_PER_PULSE = new LoggedTunableValue("STEER_ENCODER_METERS_PER_PULSE");
  private LoggedTunableValue ENCODER_RESOLUTION = new LoggedTunableValue("ENCODER_RESOLUTION");
  private LoggedTunableValue DRIVE_FEEDFORWARD_KS = new LoggedTunableValue("DRIVE_FEEDFORWARD_KS");
  private LoggedTunableValue DRIVE_FEEDFORWARD_KV = new LoggedTunableValue("DRIVE_FEEDFORWARD_KV");
  private LoggedTunableValue STEER_FEEDFORWARD_KS = new LoggedTunableValue("STEER_FEEDFORWARD_KS");
  private LoggedTunableValue STEER_FEEDFORWARD_KV = new LoggedTunableValue("STEER_FEEDFORWARD_KV");
  private LoggedTunableValue LOOP_PERIOD_SECONDS = new LoggedTunableValue("LOOP_PERIOD_SECONDS");
  private LoggedTunableValue WHEEL_RADIUS_METERS = new LoggedTunableValue("WHEEL_RADIUS_METERS");
  private LoggedTunableValue DRIVE_P = new LoggedTunableValue("DRIVE_P");
  private LoggedTunableValue DRIVE_I = new LoggedTunableValue("DRIVE_I");
  private LoggedTunableValue DRIVE_D = new LoggedTunableValue("DRIVE_D");
  private LoggedTunableValue STEER_P = new LoggedTunableValue("STEER_P");
  private LoggedTunableValue STEER_I = new LoggedTunableValue("STEER_I");
  private LoggedTunableValue STEER_D = new LoggedTunableValue("STEER_D");

  private SimpleMotorFeedforward driveFeedforward;
  private SimpleMotorFeedforward steerFeedforward;
  private final PIDController driveFeedback;

  private final PIDController steerFeedback;

  public Module(ModuleIO io, ModulePosition position) {
    this.io = io;
    this.position = position;
    STEER_ENCODER_METERS_PER_PULSE
        .set(360 / ((double) ENCODER_RESOLUTION.getDouble() * (double) STEER_MOTOR_GEAR_RATIO.getDouble()));
    driveFeedforward = new SimpleMotorFeedforward((double) DRIVE_FEEDFORWARD_KS.getDouble(),
        (double) DRIVE_FEEDFORWARD_KV.getDouble());
    steerFeedforward = new SimpleMotorFeedforward((double) STEER_FEEDFORWARD_KS.getDouble(),
        (double) STEER_FEEDFORWARD_KV.getDouble());
    driveFeedback = new PIDController((double) DRIVE_P.getDouble(), (double) DRIVE_I.getDouble(),
        (double) DRIVE_D.getDouble());
    steerFeedback = new PIDController((double) STEER_P.getDouble(), (double) STEER_I.getDouble(),
        (double) STEER_D.getDouble());

  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Module " + position, inputs);

    if (ENCODER_RESOLUTION.hasChanged(hashCode()) || STEER_MOTOR_GEAR_RATIO.hasChanged(hashCode())) {

      STEER_ENCODER_METERS_PER_PULSE
          .set(360 / ((double) ENCODER_RESOLUTION.getDouble() * (double) STEER_MOTOR_GEAR_RATIO.getDouble()));
    }
    if (DRIVE_FEEDFORWARD_KS.hasChanged(hashCode()) || DRIVE_FEEDFORWARD_KV.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward((double) DRIVE_FEEDFORWARD_KS.getDouble(),
          (double) DRIVE_FEEDFORWARD_KV.getDouble());
    }
    if (STEER_FEEDFORWARD_KS.hasChanged(hashCode()) || STEER_FEEDFORWARD_KV.hasChanged(hashCode())) {
      steerFeedforward = new SimpleMotorFeedforward((double) STEER_FEEDFORWARD_KS.getDouble(),
          (double) STEER_FEEDFORWARD_KV.getDouble());
    }

    if (DRIVE_P.hasChanged(hashCode()) || DRIVE_I.hasChanged(hashCode()) || DRIVE_D.hasChanged(hashCode())
        || LOOP_PERIOD_SECONDS.hasChanged(hashCode())) {
      driveFeedback.setPID((double) DRIVE_P.getDouble(), (double) DRIVE_I.getDouble(), (double) DRIVE_D.getDouble());
    }
    if (STEER_P.hasChanged(hashCode()) || STEER_I.hasChanged(hashCode()) || STEER_D.hasChanged(hashCode())) {
      steerFeedback.setPID((double) STEER_P.getDouble(), (double) STEER_I.getDouble(), (double) STEER_D.getDouble());
    }

    ENCODER_RESOLUTION.periodic();
    STEER_MOTOR_GEAR_RATIO.periodic();
    STEER_ENCODER_METERS_PER_PULSE.periodic();
    DRIVE_FEEDFORWARD_KS.periodic();
    DRIVE_FEEDFORWARD_KV.periodic();
    STEER_FEEDFORWARD_KS.periodic();
    STEER_FEEDFORWARD_KV.periodic();
    LOOP_PERIOD_SECONDS.periodic();
    WHEEL_RADIUS_METERS.periodic();
    DRIVE_P.periodic();
    DRIVE_I.periodic();
    DRIVE_D.periodic();
    STEER_P.periodic();
    STEER_I.periodic();
    STEER_D.periodic();

  }

  public void setDesiredState(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setSteerVoltage(steerFeedforward.calculate(optimizedState.angle.getRadians())
        +
        steerFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    double velocityRadPerSec = optimizedState.speedMetersPerSecond / (double) WHEEL_RADIUS_METERS.getDouble();

    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

  }

  public Rotation2d getAngle() {
    return new Rotation2d(MathUtil.angleModulus(inputs.steerAbsolutePositionRad));
  }

  public void stop() {
    io.setSteerVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setSteerBrakeMode(enabled);
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * (double) WHEEL_RADIUS_METERS.getDouble();
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * (double) WHEEL_RADIUS_METERS.getDouble();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public frc.robot.utils.ModulePosition getModulePosition() {
    return position;
  }

  public void setModulePose(Pose2d pose) {
    this.pose = pose;
  }

  public Pose2d getModulePose() {
    return pose;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d
        .fromDegrees(inputs.steerAbsolutePositionRad * (double) STEER_ENCODER_METERS_PER_PULSE.getDouble());
  }

  @Override
  public void periodic() {
    Logger.getInstance().recordOutput("Module " + position + " Velocity", getVelocityMetersPerSec());
    updateInputs();
  }

}
