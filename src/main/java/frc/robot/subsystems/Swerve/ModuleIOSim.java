package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.LoggedTunableValue;

public class ModuleIOSim implements ModuleIO {

    public LoggedTunableValue DRIVE_MOTOR_GEAR_RATIO = new LoggedTunableValue("Swerve/DRIVE_MOTOR_GEAR_RATIO",
            "DRIVE_MOTOR_GEAR_RATIO");
    public LoggedTunableValue STEER_MOTOR_GEAR_RATIO = new LoggedTunableValue("Swerve/STEER_MOTOR_GEAR_RATIO",
            "STEER_MOTOR_GEAR_RATIO");

    public LoggedTunableValue LOOP_PERIOD_SECONDS = new LoggedTunableValue("Swerve/LOOP_PERIOD_SECONDS",
            "LOOP_PERIOD_SECONDS");

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), DRIVE_MOTOR_GEAR_RATIO.getDouble(), 0.02);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), STEER_MOTOR_GEAR_RATIO.getDouble(),
            0.03);

    public double steerPositionRad = 0;
    public double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    public double driveVolts = 0;
    public double steerVolts = 0;

    public void updateInputs(ModuleIOInputs inputs) {
        updateTunableNumbers();

        driveSim.update(LOOP_PERIOD_SECONDS.getDouble());
        turnSim.update(LOOP_PERIOD_SECONDS.getDouble());

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * LOOP_PERIOD_SECONDS.getDouble();
        steerAbsolutePositionRad += angleDiffRad;
        steerPositionRad += angleDiffRad;

        while (steerAbsolutePositionRad < 0) {
            steerAbsolutePositionRad += 2.0 * Math.PI;
        }

        while (steerAbsolutePositionRad > 0) {
            steerAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * LOOP_PERIOD_SECONDS.getDouble());
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveVolts;
        inputs.driveCurrentAmps = new double[] { driveSim.getCurrentDrawAmps() };
        inputs.driveTemperatureCelcius = new double[] {};

        inputs.steerAbsolutePositionRad = steerAbsolutePositionRad;
        inputs.steerPositionRad = steerPositionRad;
        inputs.steerVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerVolts;
        inputs.steerCurrentAmps = new double[] { turnSim.getCurrentDrawAmps() };
        inputs.steerTemperatureCelcius = new double[] {};
        Unmanaged.feedEnable(20);

    }

    public void setDriveVoltage(double volts) {

        driveVolts = MathUtil.clamp(volts, -12.0, 12.0);

        driveSim.setInputVoltage(driveVolts);

    }

    public void setSteerVoltage(double volts) {

        steerVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(steerVolts);

    }

    public void updateTunableNumbers() {
        DRIVE_MOTOR_GEAR_RATIO.periodic();
        STEER_MOTOR_GEAR_RATIO.periodic();
        LOOP_PERIOD_SECONDS.periodic();
    }

}
