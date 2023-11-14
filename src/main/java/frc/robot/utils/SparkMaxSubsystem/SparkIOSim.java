package frc.robot.utils.SparkMaxSubsystem;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utils.LoggedTunableValue;

public class SparkIOSim implements SparkIO {
	private LoggedTunableValue EXAMPLE_SIM_SPARK_ID = new LoggedTunableValue("ExampleSpark/EXAMPLE_SIM_SPARK_ID",
			"EXAMPLE_SIM_SPARK_ID");
	private LoggedTunableValue SPARK_P = new LoggedTunableValue("ExampleSpark/SPARK_P", "SPARK_P");
	private LoggedTunableValue SPARK_I = new LoggedTunableValue("ExampleSpark/SPARK_I", "SPARK_I");
	private LoggedTunableValue SPARK_D = new LoggedTunableValue("ExampleSpark/SPARK_D", "SPARK_D");
	private LoggedTunableValue SPARK_FF = new LoggedTunableValue("ExampleSpark/SPARK_FF", "SPARK_FF");
	private LoggedTunableValue SPARK_IZONE = new LoggedTunableValue("ExampleSpark/SPARK_IZONE", "SPARK_IZONE");
	private LoggedTunableValue SPARK_MAX_OUTPUT = new LoggedTunableValue("ExampleSpark/SPARK_MAX_OUTPUT",
			"SPARK_MAX_OUTPUT");
	private LoggedTunableValue SPARK_MIN_OUTPUT = new LoggedTunableValue("ExampleSpark/SPARK_MIN_OUTPUT",
			"SPARK_MIN_OUTPUT");
	private LoggedTunableValue EXAMPLE_SIM_SPARK_MAX_ACCEL = new LoggedTunableValue(
			"ExampleSpark/EXAMPLE_SIM_SPARK_MAX_ACCEL", "EXAMPLE_SIM_SPARK_MAX_ACCEL");
	private LoggedTunableValue SPARK_CLOSED_LOOP_ERROR = new LoggedTunableValue("ExampleSpark/SPARK_CLOSED_LOOP_ERROR",
			"SPARK_CLOSED_LOOP_ERROR");
	private LoggedTunableValue LOOP_PERIOD_SECONDS = new LoggedTunableValue("Swerve/LOOP_PERIOD_SECONDS",
			"LOOP_PERIOD_SECONDS");
	private CANSparkMax spark = new CANSparkMax(EXAMPLE_SIM_SPARK_ID.getInteger(), CANSparkMax.MotorType.kBrushless);
	private SparkMaxPIDController pidController = spark.getPIDController();
	private RelativeEncoder encoder = spark.getEncoder();

	private double absolutePosition = Math.random() * 2048;

	public SparkIOSim() {
		pidController.setP(SPARK_P.getDouble());
		pidController.setI(SPARK_I.getDouble());
		pidController.setD(SPARK_D.getDouble());
		pidController.setFF(SPARK_FF.getDouble());
		pidController.setIZone(SPARK_IZONE.getDouble());
		pidController.setOutputRange(SPARK_MIN_OUTPUT.getDouble(), SPARK_MAX_OUTPUT.getDouble());
		pidController.setSmartMotionMaxAccel(EXAMPLE_SIM_SPARK_MAX_ACCEL.getDouble(), 0);
		pidController.setSmartMotionAllowedClosedLoopError(SPARK_CLOSED_LOOP_ERROR.getDouble(), 0);

		REVPhysicsSim.getInstance().addSparkMax(spark, DCMotor.getNEO(1));
	}

	public void updateInputs(SparkIOInputs inputs) {
		updateTunableNumbers();

		double positionDiff = encoder.getVelocity() * LOOP_PERIOD_SECONDS.getDouble();
		absolutePosition += positionDiff;

		while (absolutePosition < 0) {
			absolutePosition += 2048;
		}

		while (absolutePosition > 2048) {
			absolutePosition -= 2048;
		}

		inputs.absolutePosition = absolutePosition;
		inputs.relativePosition = encoder.getPosition();
		inputs.velocity = encoder.getVelocity() * LOOP_PERIOD_SECONDS.getDouble();
		inputs.appliedVolts = spark.getAppliedOutput();
		inputs.currentAmps = new double[] { spark.getOutputCurrent() };
		inputs.tempCelcius = new double[] { spark.getMotorTemperature() };

		REVPhysicsSim.getInstance().run();
	}

	public void setReference(double position) {
		pidController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
	}

	public void updateTunableNumbers() {
		if (EXAMPLE_SIM_SPARK_ID.hasChanged(hashCode()))
			spark = new CANSparkMax(EXAMPLE_SIM_SPARK_ID.getInteger(), CANSparkMax.MotorType.kBrushless);

		if (SPARK_P.hasChanged(hashCode()) || SPARK_I.hasChanged(hashCode()) || SPARK_D.hasChanged(hashCode())
				|| SPARK_FF.hasChanged(hashCode()) || SPARK_IZONE.hasChanged(hashCode())
				|| SPARK_MAX_OUTPUT.hasChanged(hashCode()) || SPARK_MIN_OUTPUT.hasChanged(hashCode())
				|| EXAMPLE_SIM_SPARK_MAX_ACCEL.hasChanged(hashCode()) || SPARK_CLOSED_LOOP_ERROR.hasChanged(hashCode())) {
			pidController.setP(SPARK_P.getDouble());
			pidController.setI(SPARK_I.getDouble());
			pidController.setD(SPARK_D.getDouble());
			pidController.setFF(SPARK_FF.getDouble());
			pidController.setIZone(SPARK_IZONE.getDouble());
			pidController.setOutputRange(SPARK_MIN_OUTPUT.getDouble(), SPARK_MAX_OUTPUT.getDouble());
			pidController.setSmartMotionMaxAccel(EXAMPLE_SIM_SPARK_MAX_ACCEL.getDouble(), 0);
			pidController.setSmartMotionAllowedClosedLoopError(SPARK_CLOSED_LOOP_ERROR.getDouble(), 0);
		}

		EXAMPLE_SIM_SPARK_ID.periodic();
		SPARK_P.periodic();
		SPARK_I.periodic();
		SPARK_D.periodic();
		SPARK_FF.periodic();
		SPARK_IZONE.periodic();
		SPARK_MAX_OUTPUT.periodic();
		SPARK_MIN_OUTPUT.periodic();
		EXAMPLE_SIM_SPARK_MAX_ACCEL.periodic();
		SPARK_CLOSED_LOOP_ERROR.periodic();
		LOOP_PERIOD_SECONDS.periodic();

	}

}
