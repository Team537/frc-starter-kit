package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;



public class ModuleIOSim implements ModuleIO {
    
    public static double kDriveMotorGearRatio = 7.13;
    public static double kSteerMotorGearRatio = 15.428;

    public static double loopPeriodSecs = 0.02;

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), kDriveMotorGearRatio, 0.02);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), kSteerMotorGearRatio,
            0.03);

    public double turnPositionRad = 0;
    public double turnAbsolutePositionRad = 0;
    public double driveVolts = 0;
    public double turnVolts = 0;

    public void updateInputs(ModuleIOInputs inputs) {
        Unmanaged.feedEnable(20);
        driveSim.update(loopPeriodSecs);
        turnSim.update(loopPeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * loopPeriodSecs;
        turnAbsolutePositionRad += angleDiffRad;
        turnPositionRad += angleDiffRad;

        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }

        while (turnAbsolutePositionRad > 0) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveVolts;
        inputs.driveCurrentAmps = new double[] { driveSim.getCurrentDrawAmps() };
        inputs.driveTemperatureCelcius = new double[] {};

        inputs.steerAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.steerPositionRad = turnPositionRad;
        inputs.steerVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = turnVolts;
        inputs.steerCurrentAmps = new double[] { turnSim.getCurrentDrawAmps() };
        inputs.steerTemperatureCelcius = new double[] {};

    }

    public void setDriveVolts(double volts) {
        driveVolts = MathUtil.clamp(-12, volts, 12);
        driveSim.setInputVoltage(volts + RobotController.getBatteryVoltage());

    }

    public void setTurnVolts(double volts) {
        turnVolts = MathUtil.clamp(-12, volts, 12);
        turnSim.setInputVoltage(volts + RobotController.getBatteryVoltage());

    }
}
