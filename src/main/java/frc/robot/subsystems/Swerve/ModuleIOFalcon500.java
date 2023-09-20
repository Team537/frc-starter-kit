package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.robot.utils.CtreUtils;
import frc.robot.utils.LoggedTunableValue;
import frc.robot.utils.ModulePosition;

public class ModuleIOFalcon500 implements ModuleIO {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX steerMotor;
    private CANCoder absoluteEncoder;
    private final ModulePosition position;

    public LoggedTunableValue FRONT_LEFT_DRIVE_MOTOR_ID = new LoggedTunableValue("Swerve/FRONT_LEFT_DRIVE_MOTOR_ID",
            "FRONT_LEFT_DRIVE_MOTOR_ID");
    public LoggedTunableValue FRONT_LEFT_STEER_MOTOR_ID = new LoggedTunableValue("Swerve/FRONT_LEFT_STEER_MOTOR_ID",
            "FRONT_LEFT_STEER_MOTOR_ID");
    public LoggedTunableValue FRONT_RIGHT_DRIVE_MOTOR_ID = new LoggedTunableValue("Swerve/FRONT_RIGHT_DRIVE_MOTOR_ID",
            "FRONT_RIGHT_DRIVE_MOTOR_ID");
    public LoggedTunableValue FRONT_RIGHT_STEER_MOTOR_ID = new LoggedTunableValue("Swerve/FRONT_RIGHT_STEER_MOTOR_ID",
            "FRONT_RIGHT_STEER_MOTOR_ID");
    public LoggedTunableValue BACK_LEFT_DRIVE_MOTOR_ID = new LoggedTunableValue("Swerve/BACK_LEFT_DRIVE_MOTOR_ID",
            "BACK_LEFT_DRIVE_MOTOR_ID");
    public LoggedTunableValue BACK_LEFT_STEER_MOTOR_ID = new LoggedTunableValue("Swerve/BACK_LEFT_STEER_MOTOR_ID",
            "BACK_LEFT_STEER_MOTOR_ID");
    public LoggedTunableValue BACK_RIGHT_DRIVE_MOTOR_ID = new LoggedTunableValue("Swerve/BACK_RIGHT_DRIVE_MOTOR_ID",
            "BACK_RIGHT_DRIVE_MOTOR_ID");
    public LoggedTunableValue BACK_RIGHT_STEER_MOTOR_ID = new LoggedTunableValue("Swerve/BACK_RIGHT_STEER_MOTOR_ID",
            "BACK_RIGHT_STEER_MOTOR_ID");

    public LoggedTunableValue FRONT_LEFT_DRIVE_INVERTED = new LoggedTunableValue("Swerve/FRONT_LEFT_DRIVE_INVERTED",
            "FRONT_LEFT_DRIVE_INVERTED");
    public LoggedTunableValue FRONT_LEFT_STEER_INVERTED = new LoggedTunableValue("Swerve/FRONT_LEFT_STEER_INVERTED",
            "FRONT_LEFT_STEER_INVERTED");
    public LoggedTunableValue FRONT_RIGHT_DRIVE_INVERTED = new LoggedTunableValue("Swerve/FRONT_RIGHT_DRIVE_INVERTED",
            "FRONT_RIGHT_DRIVE_INVERTED");
    public LoggedTunableValue FRONT_RIGHT_STEER_INVERTED = new LoggedTunableValue("Swerve/FRONT_RIGHT_STEER_INVERTED",
            "FRONT_RIGHT_STEER_INVERTED");
    public LoggedTunableValue BACK_LEFT_DRIVE_INVERTED = new LoggedTunableValue("Swerve/BACK_LEFT_DRIVE_INVERTED",
            "BACK_LEFT_DRIVE_INVERTED");
    public LoggedTunableValue BACK_LEFT_STEER_INVERTED = new LoggedTunableValue("Swerve/BACK_LEFT_STEER_INVERTED",
            "BACK_LEFT_STEER_INVERTED");
    public LoggedTunableValue BACK_RIGHT_DRIVE_INVERTED = new LoggedTunableValue("Swerve/BACK_RIGHT_DRIVE_INVERTED",
            "BACK_RIGHT_DRIVE_INVERTED");
    public LoggedTunableValue BACK_RIGHT_STEER_INVERTED = new LoggedTunableValue("Swerve/BACK_RIGHT_STEER_INVERTED",
            "BACK_RIGHT_STEER_INVERTED");

    public LoggedTunableValue FRONT_LEFT_CANCODER_ID = new LoggedTunableValue("Swerve/FRONT_LEFT_CANCODER_ID",
            "FRONT_LEFT_CANCODER_ID");

    public LoggedTunableValue FRONT_RIGHT_CANCODER_ID = new LoggedTunableValue("Swerve/FRONT_RIGHT_CANCODER_ID",
            "FRONT_RIGHT_CANCODER_ID");

    public LoggedTunableValue BACK_LEFT_CANCODER_ID = new LoggedTunableValue("Swerve/BACK_LEFT_CANCODER_ID",
            "BACK_LEFT_CANCODER_ID");

    public LoggedTunableValue BACK_RIGHT_CANCODER_ID = new LoggedTunableValue("Swerve/BACK_RIGHT_CANCODER_ID",
            "BACK_RIGHT_CANCODER_ID");

    public LoggedTunableValue FRONT_LEFT_CANCODER_OFFSET = new LoggedTunableValue("Swerve/FRONT_LEFT_CANCODER_OFFSET",
            "FRONT_LEFT_CANCODER_OFFSET");

    public LoggedTunableValue FRONT_RIGHT_CANCODER_OFFSET = new LoggedTunableValue("Swerve/FRONT_RIGHT_CANCODER_OFFSET",
            "FRONT_RIGHT_CANCODER_OFFSET");

    public LoggedTunableValue BACK_LEFT_CANCODER_OFFSET = new LoggedTunableValue("Swerve/BACK_LEFT_CANCODER_OFFSET",
            "BACK_LEFT_CANCODER_OFFSET");

    public LoggedTunableValue BACK_RIGHT_CANCODER_OFFSET = new LoggedTunableValue("Swerve/BACK_RIGHT_CANCODER_OFFSET",
            "BACK_RIGHT_CANCODER_OFFSET");

    public LoggedTunableValue DRIVE_MOTOR_GEAR_RATIO = new LoggedTunableValue("Swerve/DRIVE_MOTOR_GEAR_RATIO",
            "DRIVE_MOTOR_GEAR_RATIO");
    public LoggedTunableValue STEER_MOTOR_GEAR_RATIO = new LoggedTunableValue("Swerve/STEER_MOTOR_GEAR_RATIO",
            "STEER_MOTOR_GEAR_RATIO");

    private LoggedTunableValue TRACK_WIDTH_METERS = new LoggedTunableValue("Swerve/TRACK_WIDTH_METERS",
            "TRACK_WIDTH_METERS");
    private LoggedTunableValue WHEEL_RADIUS_METERS = new LoggedTunableValue("Swerve/WHEEL_RADIUS_METERS",
            "WHEEL_RADIUS_METERS");

    public static Translation2d[] MODULE_TRANSLATIONS;

    public static SwerveDriveKinematics DRIVE_KINEMATICS;

    public ModuleIOFalcon500(ModulePosition modulePosition) {
        this.position = modulePosition;
        MODULE_TRANSLATIONS = new Translation2d[] {

                new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2),

                new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2),

                new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, -TRACK_WIDTH_METERS.getDouble() / 2),

                new Translation2d(-WHEEL_RADIUS_METERS.getDouble() / 2, TRACK_WIDTH_METERS.getDouble() / 2) };

        DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        switch (position) {

            case FRONT_LEFT:
                driveMotor = new WPI_TalonFX((int) FRONT_LEFT_DRIVE_MOTOR_ID.getInteger());
                steerMotor = new WPI_TalonFX((int) FRONT_LEFT_STEER_MOTOR_ID.getInteger());
                driveMotor.setInverted((Boolean) FRONT_LEFT_DRIVE_INVERTED.getBool());
                steerMotor.setInverted((Boolean) FRONT_LEFT_STEER_INVERTED.getBool());
                absoluteEncoder = new CANCoder((int) FRONT_LEFT_CANCODER_ID.getInteger());
                absoluteEncoder.setPosition((double) FRONT_LEFT_CANCODER_OFFSET.getDouble());

                break;

            case FRONT_RIGHT:
                driveMotor = new WPI_TalonFX((int) FRONT_RIGHT_DRIVE_MOTOR_ID.getInteger());
                steerMotor = new WPI_TalonFX((int) FRONT_RIGHT_STEER_MOTOR_ID.getInteger());
                driveMotor.setInverted((Boolean) FRONT_RIGHT_DRIVE_INVERTED.getBool());
                steerMotor.setInverted((Boolean) FRONT_RIGHT_STEER_INVERTED.getBool());
                absoluteEncoder = new CANCoder((int) FRONT_RIGHT_CANCODER_ID.getInteger());
                absoluteEncoder.setPosition((double) FRONT_RIGHT_CANCODER_OFFSET.getDouble());
                break;
            case BACK_LEFT:
                driveMotor = new WPI_TalonFX((int) BACK_LEFT_DRIVE_MOTOR_ID.getInteger());
                steerMotor = new WPI_TalonFX((int) BACK_LEFT_STEER_MOTOR_ID.getInteger());
                driveMotor.setInverted((Boolean) BACK_LEFT_DRIVE_INVERTED.getBool());
                steerMotor.setInverted((Boolean) BACK_LEFT_STEER_INVERTED.getBool());
                absoluteEncoder = new CANCoder((int) BACK_LEFT_CANCODER_ID.getInteger());
                absoluteEncoder.setPosition((double) BACK_LEFT_CANCODER_OFFSET.getDouble());

                break;
            case BACK_RIGHT:
                driveMotor = new WPI_TalonFX((int) BACK_RIGHT_DRIVE_MOTOR_ID.getInteger());
                steerMotor = new WPI_TalonFX((int) BACK_RIGHT_STEER_MOTOR_ID.getInteger());
                driveMotor.setInverted((Boolean) BACK_RIGHT_DRIVE_INVERTED.getBool());
                steerMotor.setInverted((Boolean) BACK_RIGHT_STEER_INVERTED.getBool());
                absoluteEncoder = new CANCoder((int) BACK_RIGHT_CANCODER_ID.getInteger());
                absoluteEncoder.setPosition((double) BACK_RIGHT_CANCODER_OFFSET.getDouble());
                break;
            default:
                throw new RuntimeException("Invalid module index for Swerve");

        }
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());
        driveMotor.setSensorPhase(true);
        driveMotor.setSafetyEnabled(true);
        driveMotor.enableVoltageCompensation(true);

        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(CtreUtils.generateTurnMotorConfig());
        steerMotor.setSensorPhase(true);
        steerMotor.setSafetyEnabled(true);
        steerMotor.enableVoltageCompensation(true);

        CANCoderConfiguration canCoderConfig = CtreUtils.generateCanCoderConfig();
        absoluteEncoder.configAllSettings(canCoderConfig);

        // steerMotor.setSelectedSensorPosition(absoluteEncoder.getPosition() * 2048 /
        // 360);

    }

    public void updateInputs(ModuleIOInputs inputs) {
        updateTunableNumbers();

        inputs.drivePositionRad = driveMotor.getSelectedSensorPosition() * Math.PI * 2
                * (double) DRIVE_MOTOR_GEAR_RATIO.getDouble() / 2048;

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.driveVelocityRadPerSec = driveMotor.getSelectedSensorVelocity() * 10 * Math.PI * 2
                * (double) DRIVE_MOTOR_GEAR_RATIO.getDouble() / 2048;

        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { driveMotor.getSupplyCurrent() };
        inputs.driveTemperatureCelcius = new double[] { driveMotor.getTemperature() };

        inputs.steerAbsolutePositionRad = absoluteEncoder.getPosition() * 2 * Math.PI
                / (double) STEER_MOTOR_GEAR_RATIO.getDouble();
        inputs.steerPositionRad = steerMotor.getSelectedSensorPosition() * 2 * Math.PI
                / (2048 * (double) STEER_MOTOR_GEAR_RATIO.getDouble());

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.steerVelocityRadPerSec = steerMotor.getSelectedSensorVelocity() * 10 * 2 * Math.PI
                / (2048 * (double) STEER_MOTOR_GEAR_RATIO.getDouble());

        inputs.steerAppliedVolts = steerMotor.getMotorOutputVoltage() * steerMotor.getBusVoltage();
        inputs.steerCurrentAmps = new double[] { steerMotor.getSupplyCurrent() };
        inputs.steerTemperatureCelcius = new double[] { steerMotor.getTemperature() };
    }

    public void updateTunableNumbers() {

        if (FRONT_LEFT_DRIVE_MOTOR_ID.hasChanged(hashCode()) ||
                FRONT_LEFT_DRIVE_INVERTED.hasChanged(hashCode()) ||
                FRONT_LEFT_STEER_MOTOR_ID.hasChanged(hashCode()) ||
                FRONT_LEFT_STEER_INVERTED.hasChanged(hashCode()) ||
                FRONT_RIGHT_DRIVE_MOTOR_ID.hasChanged(hashCode()) ||
                FRONT_RIGHT_DRIVE_INVERTED.hasChanged(hashCode()) ||
                FRONT_RIGHT_STEER_MOTOR_ID.hasChanged(hashCode()) ||
                FRONT_RIGHT_STEER_INVERTED.hasChanged(hashCode()) ||
                BACK_LEFT_DRIVE_MOTOR_ID.hasChanged(hashCode()) ||
                BACK_LEFT_DRIVE_INVERTED.hasChanged(hashCode()) ||
                BACK_LEFT_STEER_MOTOR_ID.hasChanged(hashCode()) ||
                BACK_LEFT_STEER_INVERTED.hasChanged(hashCode()) ||
                BACK_RIGHT_DRIVE_MOTOR_ID.hasChanged(hashCode()) ||
                BACK_RIGHT_DRIVE_INVERTED.hasChanged(hashCode()) ||
                BACK_RIGHT_STEER_MOTOR_ID.hasChanged(hashCode()) ||
                BACK_RIGHT_STEER_INVERTED.hasChanged(hashCode()) ||
                FRONT_LEFT_CANCODER_ID.hasChanged(hashCode()) ||
                FRONT_LEFT_CANCODER_OFFSET.hasChanged(hashCode()) ||
                FRONT_RIGHT_CANCODER_ID.hasChanged(hashCode()) ||
                FRONT_RIGHT_CANCODER_OFFSET.hasChanged(hashCode()) ||
                BACK_LEFT_CANCODER_ID.hasChanged(hashCode()) ||
                BACK_LEFT_CANCODER_OFFSET.hasChanged(hashCode()) ||
                BACK_RIGHT_CANCODER_ID.hasChanged(hashCode()) ||
                BACK_RIGHT_CANCODER_OFFSET.hasChanged(hashCode())) {
            switch (position) {

                case FRONT_LEFT:
                    driveMotor = new WPI_TalonFX((int) FRONT_LEFT_DRIVE_MOTOR_ID.getInteger());
                    steerMotor = new WPI_TalonFX((int) FRONT_LEFT_STEER_MOTOR_ID.getInteger());
                    driveMotor.setInverted(FRONT_LEFT_DRIVE_INVERTED.getBool());
                    steerMotor.setInverted(FRONT_LEFT_STEER_INVERTED.getBool());
                    absoluteEncoder = new CANCoder((int) FRONT_LEFT_CANCODER_ID.getInteger());
                    absoluteEncoder.setPosition((double) FRONT_LEFT_CANCODER_OFFSET.getDouble());
                    break;

                case FRONT_RIGHT:
                    driveMotor = new WPI_TalonFX((int) FRONT_RIGHT_DRIVE_MOTOR_ID.getInteger());
                    steerMotor = new WPI_TalonFX((int) FRONT_RIGHT_STEER_MOTOR_ID.getInteger());
                    driveMotor.setInverted(FRONT_RIGHT_DRIVE_INVERTED.getBool());
                    steerMotor.setInverted(FRONT_RIGHT_STEER_INVERTED.getBool());
                    absoluteEncoder = new CANCoder((int) FRONT_RIGHT_CANCODER_ID.getInteger());
                    absoluteEncoder.setPosition((double) FRONT_RIGHT_CANCODER_OFFSET.getDouble());
                    break;
                case BACK_LEFT:
                    driveMotor = new WPI_TalonFX((int) BACK_LEFT_DRIVE_MOTOR_ID.getInteger());
                    steerMotor = new WPI_TalonFX((int) BACK_LEFT_STEER_MOTOR_ID.getInteger());
                    driveMotor.setInverted(BACK_LEFT_DRIVE_INVERTED.getBool());
                    steerMotor.setInverted(BACK_LEFT_STEER_INVERTED.getBool());
                    absoluteEncoder = new CANCoder((int) BACK_LEFT_CANCODER_ID.getInteger());
                    absoluteEncoder.setPosition((double) BACK_LEFT_CANCODER_OFFSET.getDouble());

                    break;
                case BACK_RIGHT:
                    driveMotor = new WPI_TalonFX((int) BACK_RIGHT_DRIVE_MOTOR_ID.getInteger());
                    steerMotor = new WPI_TalonFX((int) BACK_RIGHT_STEER_MOTOR_ID.getInteger());
                    driveMotor.setInverted(BACK_RIGHT_DRIVE_INVERTED.getBool());
                    steerMotor.setInverted(BACK_RIGHT_STEER_INVERTED.getBool());
                    absoluteEncoder = new CANCoder((int) BACK_RIGHT_CANCODER_ID.getInteger());
                    absoluteEncoder.setPosition((double) BACK_RIGHT_CANCODER_OFFSET.getDouble());
                    break;
                default:
                    throw new RuntimeException("Invalid module index for Swerve");

            }

        }

        if (WHEEL_RADIUS_METERS.hasChanged(hashCode()) || TRACK_WIDTH_METERS.hasChanged(hashCode())) {
            MODULE_TRANSLATIONS = new Translation2d[] {

                    new Translation2d(-(Double) WHEEL_RADIUS_METERS.getDouble() / 2,
                            (Double) TRACK_WIDTH_METERS.getDouble() / 2),

                    new Translation2d(-(Double) WHEEL_RADIUS_METERS.getDouble() / 2,
                            (Double) TRACK_WIDTH_METERS.getDouble() / 2),

                    new Translation2d(-(Double) WHEEL_RADIUS_METERS.getDouble() / 2,
                            -(Double) TRACK_WIDTH_METERS.getDouble() / 2),

                    new Translation2d(-(Double) WHEEL_RADIUS_METERS.getDouble() / 2,
                            (Double) TRACK_WIDTH_METERS.getDouble() / 2) };

            DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        }

        FRONT_LEFT_DRIVE_MOTOR_ID.periodic();
        FRONT_LEFT_STEER_MOTOR_ID.periodic();
        FRONT_RIGHT_DRIVE_MOTOR_ID.periodic();
        FRONT_RIGHT_STEER_MOTOR_ID.periodic();
        BACK_LEFT_DRIVE_MOTOR_ID.periodic();
        BACK_LEFT_STEER_MOTOR_ID.periodic();
        BACK_RIGHT_DRIVE_MOTOR_ID.periodic();
        BACK_RIGHT_STEER_MOTOR_ID.periodic();
        DRIVE_MOTOR_GEAR_RATIO.periodic();
        STEER_MOTOR_GEAR_RATIO.periodic();
        FRONT_LEFT_DRIVE_INVERTED.periodic();
        FRONT_LEFT_STEER_INVERTED.periodic();
        FRONT_RIGHT_DRIVE_INVERTED.periodic();
        FRONT_RIGHT_STEER_INVERTED.periodic();
        BACK_LEFT_DRIVE_INVERTED.periodic();
        BACK_LEFT_STEER_INVERTED.periodic();
        BACK_RIGHT_DRIVE_INVERTED.periodic();
        BACK_RIGHT_STEER_INVERTED.periodic();

        FRONT_LEFT_CANCODER_ID.periodic();
        FRONT_RIGHT_CANCODER_ID.periodic();
        BACK_LEFT_CANCODER_ID.periodic();
        BACK_RIGHT_CANCODER_ID.periodic();

        FRONT_LEFT_CANCODER_OFFSET.periodic();
        FRONT_RIGHT_CANCODER_OFFSET.periodic();
        BACK_LEFT_CANCODER_OFFSET.periodic();
        BACK_RIGHT_CANCODER_OFFSET.periodic();

        DRIVE_MOTOR_GEAR_RATIO.periodic();
        STEER_MOTOR_GEAR_RATIO.periodic();

        WHEEL_RADIUS_METERS.periodic();
        TRACK_WIDTH_METERS.periodic();

    }

    public void setDriveVoltage(double volts) {

        driveMotor.set(ControlMode.PercentOutput, volts / 12);

    }

    public void setSteerVoltage(double volts) {

        steerMotor.set(ControlMode.PercentOutput, volts / 12);

    }

    public void setDriveBrakeMode(boolean brake) {

        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setSteerBrakeMode(boolean brake) {

        steerMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

}
