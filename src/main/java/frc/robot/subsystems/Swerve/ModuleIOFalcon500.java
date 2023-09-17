package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModulePosition;

public class ModuleIOFalcon500 implements ModuleIO{
    private final WPI_TalonFX m_drive;
    private final WPI_TalonFX m_turn;

  

    public static int kFrontLeftDrive = 0;
    public static int kFrontLeftTurn = 1;
    public static int kFrontRightDrive = 2;
    public static int kFrontRightTurn = 3;
    public static int kBackLeftDrive = 4;
    public static int kBackLeftTurn = 5;
    public static int kBackRightDrive = 6;
    public static int kBackRightTurn = 7;
    public static double kDriveMotorGearRatio = 7.13;
    public static double kSteerMotorGearRatio = 15.428;
    public static double kMaxSpeed = 10;
    public static double trackwidthMeters = 0.415;
   
    private static double wheelBase = 0.415;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
    
    new Translation2d(-wheelBase / 2, trackwidthMeters / 2),
    
    new Translation2d(-wheelBase / 2, trackwidthMeters / 2),
    
    new Translation2d(wheelBase / 2, -trackwidthMeters / 2),
    
    new Translation2d(wheelBase / 2, trackwidthMeters / 2)};
    public static final double FEED_FORWARD_STATIC_GAIN = 0;
    public static final double FEED_FORWARD_VELOCITY_GAIN = 0;
    public static final double FEED_FORWARD_ACCELERATION_GAIN = 0;
    public static final double TURN_ENCODER_METERS_PER_PULSE = 0;

    public static SwerveDriveKinematics kDriveKinematics =  new SwerveDriveKinematics(moduleTranslations);

    public static double loopPeriodSecs = 0.02;
   






    public ModuleIOFalcon500(ModulePosition modulePosition) {

        switch (modulePosition) {

            case FRONT_LEFT:
                m_drive = new WPI_TalonFX(kFrontLeftDrive);
                m_turn = new WPI_TalonFX(kFrontLeftTurn);
                m_drive.setInverted(false);
                break;

            case FRONT_RIGHT:
                m_drive = new WPI_TalonFX(kFrontRightDrive);
                m_turn = new WPI_TalonFX(kFrontRightTurn);
                m_drive.setInverted(true);
                break;
            case BACK_LEFT:
                m_drive = new WPI_TalonFX(kBackLeftDrive);
                m_turn = new WPI_TalonFX(kBackLeftTurn);
                m_drive.setInverted(true);
                break;
            case BACK_RIGHT:
                m_drive = new WPI_TalonFX(kBackRightDrive);
                m_turn = new WPI_TalonFX(kBackRightTurn);
                m_drive.setInverted(false);
                break;
            default:
                throw new RuntimeException("Invalid module index for Swerve");

        }
        m_drive.configFactoryDefault();
        m_drive.configAllSettings(CtreUtils.generateDriveMotorConfig());
        m_drive.setSensorPhase(true);
        m_drive.setSafetyEnabled(true);
        m_drive.enableVoltageCompensation(true);

        m_turn.configFactoryDefault();
        m_turn.configAllSettings(CtreUtils.generateTurnMotorConfig());

    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = m_drive.getSelectedSensorPosition() * Math.PI * 2
                * kDriveMotorGearRatio / 2048;

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.driveVelocityRadPerSec = m_drive.getSelectedSensorVelocity() * 10 * Math.PI * 2
                * kDriveMotorGearRatio / 2048;

        inputs.driveAppliedVolts = m_drive.getMotorOutputVoltage() * m_drive.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { m_drive.getSupplyCurrent() };
        inputs.driveTemperatureCelcius = new double[] { m_drive.getTemperature() };

        inputs.steerAbsolutePositionRad = 0; // Temporary, wait until Mag Encoder Implementation
        inputs.steerPositionRad = m_turn.getSelectedSensorPosition() * 2 * Math.PI
                / (2048 * kSteerMotorGearRatio);

        // Multiply 10 because Selected Sensor Velocity is measured per 100 ms
        inputs.steerVelocityRadPerSec = m_turn.getSelectedSensorVelocity() * 10 * 2 * Math.PI
                / (2048 * kSteerMotorGearRatio);

        inputs.steerAppliedVolts = m_turn.getMotorOutputVoltage() * m_turn.getBusVoltage();
        inputs.steerCurrentAmps = new double[] { m_turn.getSupplyCurrent() };
        inputs.steerTemperatureCelcius = new double[] { m_turn.getTemperature() };
    }

    public void setDriveVolts(double volts) {

        m_drive.set(ControlMode.PercentOutput, volts / 12);

    }

    public void setTurnVolts(double volts) {

        m_turn.set(ControlMode.PercentOutput, volts / 12);

    }

    public void setDriveBrake(boolean brake) {

        m_drive.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setTurnBrake(boolean brake) {

        m_turn.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }


}
