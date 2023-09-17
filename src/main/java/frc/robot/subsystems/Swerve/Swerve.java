package frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModulePosition;

public class Swerve extends SubsystemBase {

  

        private Module frontLeftModule;
        private Module frontRightModule;
        private Module backLeftModule;
        private Module backRightModule;

        private double MAX_SPEED_METERS_PER_SECOND  = 5.0;
        private static double wheelBase = 0.415;
    public static double trackwidthMeters = 0.415;

      private HashMap<ModulePosition, Module>   swerveModules = new HashMap<ModulePosition, Module>();

      private final GyroIO gyroIO;
      private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

      private PIDController xPIDController = new PIDController(1, 0, 0);

      private PIDController yPIDController = new PIDController(1, 0, 0);
      private ProfiledPIDController profiledTurnController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
      private PIDController turnController = new PIDController(1, 0,0);

      public static final Map<ModulePosition, Translation2d> MODULE_TRANSLATIONS = Map.of(
        ModulePosition.FRONT_LEFT,
        new Translation2d(-wheelBase / 2,trackwidthMeters / 2),
        ModulePosition.FRONT_RIGHT,
        new Translation2d(-wheelBase / 2, -trackwidthMeters / 2),
        ModulePosition.BACK_LEFT,
        new Translation2d(wheelBase / 2, -trackwidthMeters / 2),
        ModulePosition.BACK_RIGHT,
        new Translation2d(wheelBase / 2, trackwidthMeters/ 2));

        private final SwerveDrivePoseEstimator odometry;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        ModuleMap.orderedValues(MODULE_TRANSLATIONS, new Translation2d[0]));
    private static final double MAX_ROTATION_RADIANS_PER_SECOND = Math.PI * 2.0;

      public Swerve(GyroIO gyroIO,  ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

        this.gyroIO = gyroIO;

        frontLeftModule = new Module(flModuleIO, ModulePosition.FRONT_LEFT  );
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
            ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), MAX_SPEED_METERS_PER_SECOND);
        
        for (Module module : ModuleMap.orderedValuesList(swerveModules))
          module.setDesiredState(moduleStates.get(module.getModulePosition()));
    
      }

      public void drive(
        double drive,
        double strafe,
        double rotation,
        boolean isFieldRelative) {
      drive *= MAX_SPEED_METERS_PER_SECOND;
      strafe *= MAX_SPEED_METERS_PER_SECOND;
      rotation *= MAX_ROTATION_RADIANS_PER_SECOND;
  
      // Chassis Speed
      ChassisSpeeds chassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          drive, strafe, rotation, getHeadingRotation2d())
          : new ChassisSpeeds(drive, strafe, rotation);
  

      // Module States
      Map<ModulePosition, SwerveModuleState> moduleStates = ModuleMap
          .of(SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  
        Logger.getInstance().recordOutput("Module States", moduleStates.toString());

      setSwerveModuleStatesMap(moduleStates);
  
     
    }
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromRadians(gyroInputs.yawPositionRad);
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

      @Override 
      public void periodic() {
        updateOdometry();
        Logger.getInstance().recordOutput("Pose", getPoseMeters().toString());

        
        gyroIO.updateInputs(gyroInputs);
        
        Logger.getInstance().processInputs("Gyro", gyroInputs);
        
      }
}
