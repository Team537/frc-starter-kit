// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ModulePosition;

import java.util.Map;

/**
 * 
 * Simulates Field in Simulations
 */

public class FieldSim {
	private final Swerve swerve;

	private final Field2d field2d = new Field2d();

	private final Map<ModulePosition, Pose2d> m_swerveModulePoses = ModuleMap.of(new Pose2d(), new Pose2d(), new Pose2d(),
			new Pose2d());

	public FieldSim(Swerve swerve) {
		this.swerve = swerve;
	}

	public void initSim() {
	}

	public Field2d getField2d() {
		return field2d;
	}

	public void setTrajectory(Trajectory trajectory) {
		field2d.getObject("trajectory").setTrajectory(trajectory);
	}

	public void resetRobotPose(Pose2d pose) {
		field2d.setRobotPose(pose);
	}

	private void updateRobotPoses() {
		field2d.setRobotPose(swerve.getPoseMeters());

		for (ModulePosition i : ModulePosition.values()) {
			Translation2d updatedPositions = swerve.getModuleTranslations()
					.get(i)
					.rotateBy(swerve.getPoseMeters().getRotation())
					.plus(swerve.getPoseMeters().getTranslation());
			m_swerveModulePoses.put(
					i,
					new Pose2d(
							updatedPositions,
							swerve
									.getModule(i)
									.getHeadingRotation2d()
									.plus(swerve.getHeadingRotation2d())));
		}

		field2d
				.getObject("Swerve Modules")
				.setPoses(ModuleMap.orderedValues(m_swerveModulePoses, new Pose2d[0]));
	}

	public void periodic() {
		updateRobotPoses();

		SmartDashboard.putData("Field2d", field2d);
	}

}
