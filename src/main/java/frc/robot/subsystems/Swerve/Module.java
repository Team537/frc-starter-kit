package frc.robot.subsystems.Swerve;



import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.ModuleIOFalcon500.ModulePosition;

public class Module extends SubsystemBase {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModulePosition position;
    
    private static double wheelBase = 0.415;
    public static double loopPeriodSecs = 0.02;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController driveFeedback =
      new PIDController(0.0, 0.0, 0.0, loopPeriodSecs);
  private final PIDController steerFeedback =
      new PIDController(0.0, 0.0, 0.0, loopPeriodSecs);

    public Module(ModuleIO io, ModulePosition position) {
        this.io = io;
        this.position = position;
       
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void setDesiredState(SwerveModuleState state){
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        io.setSteerVoltage(
        steerFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

        double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelBase;
   
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
        return inputs.drivePositionRad * wheelBase;
      }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelBase;
      }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
      }

      public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
      }

      public ModulePosition getModulePosition() {
        return position;
      }
    



}
