package frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;
import frc.robot.utils.LoggedTunableValue;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModulePosition;

public class Swerve extends SubsystemBase {

  private Module frontLeftModule;
  private Module frontRightModule;
  private Module backLeftModule;
  private Module backRightModule;

  private LoggedTunableValue MAX_SPEED_METERS_PER_SECOND = new LoggedTunableValue("Swerve/MAX_SPEED_METERS_PER_SECOND",
      "MAX_SPEED_METERS_PER_SECOND");
  private LoggedTunableValue MAX_ROTATION_RADIANS_PER_SECOND = new LoggedTunableValue(
      "Swerve/MAX_ROTATION_RADIANS_PER_SECOND",
      "MAX_ROTATION_RADIANS_PER_SECOND");
  private LoggedTunableValue WHEEL_RADIUS_METERS = new LoggedTunableValue("Swerve/WHEEL_RADIUS_METERS",
      "WHEEL_RADIUS_METERS");
  private LoggedTunableValue TRACK_WIDTH_METERS = new LoggedTunableValue("Swerve/TRACK_WIDTH_METERS",
      "TRACK_WIDTH_METERS");

  private double simYaw = 0;

  private HashMap<ModulePosition, Module> swerveModules = new HashMap<ModulePosition, Module>();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  public Map<ModulePosition, Translation2d> MODULE_TRANSLATIONS;

  private SwerveDrivePoseEstimator odometry;

  public SwerveDriveKinematics SWERVE_KINEMATICS;

  public Swerve(GyroIO gyroIO, ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    MODULE_TRANSLATIONS = Map.of(
        ModulePosition.FRONT_LEFT,
        new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2),
        ModulePosition.FRONT_RIGHT,
        new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, -TRACK_WIDTH_METERS.getDouble() / 2),
        ModulePosition.BACK_LEFT,
        new Translation2d(WHEEL_RADIUS_METERS.getDouble() / 2, -TRACK_WIDTH_METERS.getDouble() / 2),
        ModulePosition.BACK_RIGHT,
        new Translation2d(WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2));

    SWERVE_KINEMATICS = new SwerveDriveKinematics(
        ModuleMap.orderedValues(MODULE_TRANSLATIONS, new Translation2d[0]));

    this.gyroIO = gyroIO;

    frontLeftModule = new Module(flModuleIO, ModulePosition.FRONT_LEFT);
    frontRightModule = new Module(frModuleIO, ModulePosition.FRONT_RIGHT);
    backLeftModule = new Module(blModuleIO, ModulePosition.BACK_LEFT);
    backRightModule = new Module(brModuleIO, ModulePosition.BACK_RIGHT);

    swerveModules = new HashMap<ModulePosition, Module>();
    swerveModules.put(ModulePosition.FRONT_LEFT, frontLeftModule);
    swerveModules.put(ModulePosition.FRONT_RIGHT, frontRightModule);
    swerveModules.put(ModulePosition.BACK_LEFT, backLeftModule);
    swerveModules.put(ModulePosition.BACK_RIGHT, backRightModule);

    odometry = new SwerveDrivePoseEstimator(
        SWERVE_KINEMATICS,
        getHeadingRotation2d(),
        getModulePositions(),
        new Pose2d());

  }

  public void setSwerveModuleStatesMap(Map<ModulePosition, SwerveModuleState> moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), MAX_SPEED_METERS_PER_SECOND.getDouble());

    for (Module module : ModuleMap.orderedValuesList(swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()));

  }

  public SwerveDriveKinematics getSwerveKinematics() {
    return SWERVE_KINEMATICS;
  }

  public void drive(
      double drive,
      double strafe,
      double rotation,
      boolean isFieldRelative) {
    drive *= MAX_SPEED_METERS_PER_SECOND.getDouble();
    strafe *= MAX_SPEED_METERS_PER_SECOND.getDouble();
    rotation *= MAX_ROTATION_RADIANS_PER_SECOND.getDouble();

    // Chassis Speed
    ChassisSpeeds chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        drive, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(drive, strafe, rotation);

    // Module States
    Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
        .of(SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));

    setSwerveModuleStatesMap(moduleStates);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromRadians(gyroInputs.yawPositionRad);
  }

  public void stop() {
    for (Module module : ModuleMap.orderedValuesList(swerveModules)) {
      module.stop();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public Pose2d getPoseMeters() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(),
        pose);
  }

  public void updateOdometry() {
    odometry.update(
        getHeadingRotation2d(),
        getModulePositions());

    for (Module module : ModuleMap.orderedValuesList(swerveModules)) {
      Translation2d modulePositionFromChassis = MODULE_TRANSLATIONS
          .get(module.getModulePosition())
          .rotateBy(getHeadingRotation2d())
          .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : swerveModules.keySet()) {
      map.put(i, swerveModules.get(i).getState());
    }
    return map;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECOND.getDouble());

    for (Module module : ModuleMap.orderedValuesList(swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()]);
  }

  public void updateTunableNumbers() {

    if (TRACK_WIDTH_METERS.hasChanged(hashCode()) || WHEEL_RADIUS_METERS.hasChanged(hashCode())) {
      MODULE_TRANSLATIONS = Map.of(
          ModulePosition.FRONT_LEFT,
          new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2),
          ModulePosition.FRONT_RIGHT,
          new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, -TRACK_WIDTH_METERS.getDouble() / 2),
          ModulePosition.BACK_LEFT,
          new Translation2d(WHEEL_RADIUS_METERS.getDouble() / 2, -TRACK_WIDTH_METERS.getDouble() / 2),
          ModulePosition.BACK_RIGHT,
          new Translation2d(WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2));

      SWERVE_KINEMATICS = new SwerveDriveKinematics(
          ModuleMap.orderedValues(MODULE_TRANSLATIONS, new Translation2d[0]));

      // odometry = new SwerveDrivePoseEstimator(
      // SWERVE_KINEMATICS,
      // getHeadingRotation2d(),
      // getModulePositions(),
      // new Pose2d());
    }

    MAX_SPEED_METERS_PER_SECOND.periodic();
    MAX_ROTATION_RADIANS_PER_SECOND.periodic();
    TRACK_WIDTH_METERS.periodic();
    WHEEL_RADIUS_METERS.periodic();
  }

  @Override
  public void periodic() {
    updateTunableNumbers();
    ChassisSpeeds chassisSpeed = SWERVE_KINEMATICS.toChassisSpeeds(
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    updateOdometry();
    // Logger.getInstance().recordOutput("Pose", getPoseMeters().toString());
    SmartDashboard.putNumber("Odo X", odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Odo Y", odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odo Yaw", odometry.getEstimatedPosition().getRotation().getDegrees());

    SmartDashboard.putString("Gyro Angle", getHeadingRotation2d().toString());
    Unmanaged.feedEnable(20);
    simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
    gyroIO.setHeading(simYaw);

    gyroIO.updateInputs(gyroInputs);

    Logger.getInstance().recordOutput("Swerve Pose", getPoseMeters());
    Logger.getInstance().recordOutput("ModuleStates",
        ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));
    Logger.getInstance().processInputs("Gyro", gyroInputs);

  }
}