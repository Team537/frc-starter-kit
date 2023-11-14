package frc.robot.utils.SparkMaxSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface SparkIO {

	@AutoLog
	public static class SparkIOInputs {

		public double absolutePosition;
		public double relativePosition;
		public double velocity;
		public double appliedVolts;
		public double[] currentAmps;
		public double[] tempCelcius;
	}

	public default void updateInputs(SparkIOInputs inputs) {
	}

	public default void setReference(double position) {
	}

}
