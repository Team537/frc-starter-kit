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
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ModulePosition;


public class Module extends SubsystemBase {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModulePosition position;
    private Pose2d pose = new Pose2d();
    
    // public static final double TURNING_MOTOR_GEAR_RATIO = 15.428;
    // public static final int ENCODER_RESOLUTION = 2048;
    // public static final double TURN_ENCODER_METERS_PER_PULSE =

    //     360 / (ENCODER_RESOLUTION * TURNING_MOTOR_GEAR_RATIO);

    private LoggedTunableNumber TURNING_MOTOR_GEAR_RATIO = new LoggedTunableNumber("TURNING_MOTOR_GEAR_RATIO");
    private LoggedTunableNumber TURN_ENCODER_METERS_PER_PULSE = new LoggedTunableNumber("TURN_ENCODER_METERS_PER_PULSE");
    private LoggedTunableNumber ENCODER_RESOLUTION = new LoggedTunableNumber("ENCODER_RESOLUTION");
    private LoggedTunableNumber DRIVE_FEEDFORWARD_KS = new LoggedTunableNumber("DRIVE_FEEDFORWARD_KS");
    private LoggedTunableNumber DRIVE_FEEDFORWARD_KV = new LoggedTunableNumber("DRIVE_FEEDFORWARD_KV");
    private LoggedTunableNumber STEER_FEEDFORWARD_KS = new LoggedTunableNumber("STEER_FEEDFORWARD_KS");
    private LoggedTunableNumber STEER_FEEDFORWARD_KV = new LoggedTunableNumber("STEER_FEEDFORWARD_KV");
    private LoggedTunableNumber LOOP_PERIOD_SECONDS = new LoggedTunableNumber("LOOP_PERIOD_SECONDS");
    private LoggedTunableNumber WHEEL_RADIUS = new LoggedTunableNumber("WHEEL_RADIUS");
    private LoggedTunableNumber DRIVE_P = new LoggedTunableNumber("DRIVE_P");
    private LoggedTunableNumber DRIVE_I = new LoggedTunableNumber("DRIVE_I");
    private LoggedTunableNumber DRIVE_D = new LoggedTunableNumber("DRIVE_D");
    private LoggedTunableNumber STEER_P = new LoggedTunableNumber("STEER_P");
    private LoggedTunableNumber STEER_I = new LoggedTunableNumber("STEER_I");
    private LoggedTunableNumber STEER_D = new LoggedTunableNumber("STEER_D");


    private SimpleMotorFeedforward driveFeedforward;
    private SimpleMotorFeedforward steerFeedforward;
    private final PIDController driveFeedback;

    private final PIDController steerFeedback;

      

    public Module(ModuleIO io, ModulePosition position) {
        this.io = io;
        this.position = position;
        TURN_ENCODER_METERS_PER_PULSE.set(360 / ( (double) ENCODER_RESOLUTION.get() * (double)  TURNING_MOTOR_GEAR_RATIO.get()));
        driveFeedforward = new SimpleMotorFeedforward((double)  DRIVE_FEEDFORWARD_KS.get(),(double)  DRIVE_FEEDFORWARD_KV.get());
        steerFeedforward = new SimpleMotorFeedforward((double)  STEER_FEEDFORWARD_KS.get(),(double)  STEER_FEEDFORWARD_KV.get());
        driveFeedback = new PIDController((double) DRIVE_P.get(),(double)  DRIVE_I.get(),(double)  DRIVE_D.get());
        steerFeedback = new PIDController((double) STEER_P.get(),(double)  STEER_I.get(),(double)  STEER_D.get());
       
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Module " + position, inputs);

        if(ENCODER_RESOLUTION.hasChanged(hashCode()) || TURNING_MOTOR_GEAR_RATIO.hasChanged(hashCode())){
          
            TURN_ENCODER_METERS_PER_PULSE.set(360 / ((double) ENCODER_RESOLUTION.get() *(double)  TURNING_MOTOR_GEAR_RATIO.get()));
        }
        if(DRIVE_FEEDFORWARD_KS.hasChanged(hashCode()) || DRIVE_FEEDFORWARD_KV.hasChanged(hashCode())){
            driveFeedforward = new SimpleMotorFeedforward((double) DRIVE_FEEDFORWARD_KS.get(), (double) DRIVE_FEEDFORWARD_KV.get());
        }
        if(STEER_FEEDFORWARD_KS.hasChanged(hashCode()) || STEER_FEEDFORWARD_KV.hasChanged(hashCode())){
            steerFeedforward = new SimpleMotorFeedforward((double) STEER_FEEDFORWARD_KS.get(),(double)  STEER_FEEDFORWARD_KV.get());
        }

        if(DRIVE_P.hasChanged(hashCode()) || DRIVE_I.hasChanged(hashCode()) || DRIVE_D.hasChanged(hashCode()) || LOOP_PERIOD_SECONDS.hasChanged(hashCode())){
            driveFeedback.setPID((double) DRIVE_P.get(),(double)  DRIVE_I.get(),(double)  DRIVE_D.get());
        }
        if(STEER_P.hasChanged(hashCode()) || STEER_I.hasChanged(hashCode()) || STEER_D.hasChanged(hashCode())){
            steerFeedback.setPID((double) STEER_P.get(),(double)  STEER_I.get(),(double)  STEER_D.get());
        }

        ENCODER_RESOLUTION.periodic();
        TURNING_MOTOR_GEAR_RATIO.periodic();
        TURN_ENCODER_METERS_PER_PULSE.periodic();
        DRIVE_FEEDFORWARD_KS.periodic();
        DRIVE_FEEDFORWARD_KV.periodic();
        STEER_FEEDFORWARD_KS.periodic();
        STEER_FEEDFORWARD_KV.periodic();
        LOOP_PERIOD_SECONDS.periodic();
        WHEEL_RADIUS.periodic();
        DRIVE_P.periodic();
        DRIVE_I.periodic();
        DRIVE_D.periodic();
        STEER_P.periodic();
        STEER_I.periodic();
        STEER_D.periodic();


      
        
    }

    public void setDesiredState(SwerveModuleState state){
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

       
        
        io.setSteerVoltage( steerFeedforward.calculate(optimizedState.angle.getRadians())
            +
        steerFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

        double velocityRadPerSec = optimizedState.speedMetersPerSecond /(double)  WHEEL_RADIUS.get();
   
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
        return inputs.drivePositionRad *(double)  WHEEL_RADIUS.get();
      }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * (double) WHEEL_RADIUS.get();
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
        return Rotation2d.fromDegrees(inputs.steerAbsolutePositionRad * (double) TURN_ENCODER_METERS_PER_PULSE.get() );
      }

      @Override
        public void periodic() {
         
            updateInputs();
        }



}
