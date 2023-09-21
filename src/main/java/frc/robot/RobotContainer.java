package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.config.YAMLDataHolder;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.ModuleIO;
import frc.robot.subsystems.Swerve.ModuleIOFalcon500;
import frc.robot.subsystems.Swerve.ModuleIOSim;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.utils.LoggedTunableValue;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.RobotMode;

public class RobotContainer {
        private Swerve swerve;
        public static RobotMode mode = RobotMode.SIM;
        private XboxController controller;
        private YAMLDataHolder yamlDataHolder;
        private FieldSim fieldSim;
        private SwerveAutoBuilder swerveAutoBuilder;
        private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.3);
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));

        private LoggedTunableValue PATH_PLANNER_DRIVE_P = new LoggedTunableValue("PathPlanner/PATH_PLANNER_DRIVE_P",
                        "PATH_PLANNER_DRIVE_P");
        private LoggedTunableValue PATH_PLANNER_DRIVE_I = new LoggedTunableValue("PathPlanner/PATH_PLANNER_DRIVE_I",
                        "PATH_PLANNER_DRIVE_I");
        private LoggedTunableValue PATH_PLANNER_DRIVE_D = new LoggedTunableValue("PathPlanner/PATH_PLANNER_DRIVE_D",
                        "PATH_PLANNER_DRIVE_D");
        private LoggedTunableValue PATH_PLANNER_STEER_P = new LoggedTunableValue("PathPlanner/PATH_PLANNER_STEER_P",
                        "PATH_PLANNER_STEER_P");
        private LoggedTunableValue PATH_PLANNER_STEER_I = new LoggedTunableValue("PathPlanner/PATH_PLANNER_STEER_I",
                        "PATH_PLANNER_STEER_I");
        private LoggedTunableValue PATH_PLANNER_STEER_D = new LoggedTunableValue("PathPlanner/PATH_PLANNER_STEER_D",
                        "PATH_PLANNER_STEER_D");
        private LoggedTunableValue CONTROLLER_1_PORT = new LoggedTunableValue("Controllers/CONTROLLER_1_PORT",
                        "CONTROLLER_1_PORT");

        HashMap<String, Command> eventMap = new HashMap<>();

        public RobotContainer() {

                yamlDataHolder = YAMLDataHolder.getInstance();
                controller = new XboxController(CONTROLLER_1_PORT.getInteger());
                switch (mode) {
                        case REAL:
                                swerve = new Swerve(new GyroIOPigeon2(),
                                                new ModuleIOFalcon500(ModulePosition.FRONT_LEFT),
                                                new ModuleIOFalcon500(ModulePosition.FRONT_RIGHT),
                                                new ModuleIOFalcon500(ModulePosition.BACK_LEFT),
                                                new ModuleIOFalcon500(ModulePosition.BACK_RIGHT));

                                break;
                        case SIM:
                                swerve = new Swerve(new GyroIOPigeon2(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());
                                break;
                        default:
                                swerve = new Swerve(new GyroIO() {
                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });

                                break;
                }

                fieldSim = new FieldSim(swerve);

                swerveAutoBuilder = new SwerveAutoBuilder(
                                swerve::getPoseMeters,
                                swerve::resetOdometry,
                                swerve.getSwerveKinematics(),
                                new PIDConstants(PATH_PLANNER_DRIVE_P.getDouble(), PATH_PLANNER_DRIVE_I.getDouble(),
                                                PATH_PLANNER_DRIVE_D.getDouble()),
                                new PIDConstants(PATH_PLANNER_STEER_P.getDouble(), PATH_PLANNER_STEER_I.getDouble(),
                                                PATH_PLANNER_STEER_D.getDouble()),
                                swerve::setSwerveModuleStates,
                                eventMap,
                                true,
                                swerve);

                swerve.setDefaultCommand(

                                new SwerveDriveCommand(swerve,
                                                () -> -controller.getLeftY(),
                                                () -> controller.getLeftX(),
                                                () -> -controller.getRightX(), true)

                );

        }

        public void onDisable() {
                swerve.stop();
                yamlDataHolder.saveData();

        }

        public void periodic() {

                fieldSim.periodic();

                if (PATH_PLANNER_DRIVE_D.hasChanged(hashCode()) ||
                                PATH_PLANNER_DRIVE_I.hasChanged(hashCode())
                                || PATH_PLANNER_DRIVE_P.hasChanged(hashCode()) ||
                                PATH_PLANNER_STEER_D.hasChanged(hashCode())
                                || PATH_PLANNER_STEER_I.hasChanged(hashCode()) ||
                                PATH_PLANNER_STEER_P.hasChanged(hashCode())) {
                        swerveAutoBuilder = new SwerveAutoBuilder(
                                        swerve::getPoseMeters,
                                        swerve::resetOdometry,
                                        swerve.getSwerveKinematics(),
                                        new PIDConstants(PATH_PLANNER_DRIVE_P.getDouble(),
                                                        PATH_PLANNER_DRIVE_I.getDouble(),
                                                        PATH_PLANNER_DRIVE_D.getDouble()),
                                        new PIDConstants(PATH_PLANNER_STEER_P.getDouble(),
                                                        PATH_PLANNER_STEER_I.getDouble(),
                                                        PATH_PLANNER_STEER_D.getDouble()),
                                        swerve::setSwerveModuleStates,
                                        eventMap,
                                        true,
                                        swerve);
                }

                if (CONTROLLER_1_PORT.hasChanged(hashCode()))
                        controller = new XboxController(CONTROLLER_1_PORT.getInteger());

                PATH_PLANNER_DRIVE_D.periodic();
                PATH_PLANNER_DRIVE_I.periodic();
                PATH_PLANNER_DRIVE_P.periodic();
                PATH_PLANNER_STEER_D.periodic();
                PATH_PLANNER_STEER_I.periodic();
                PATH_PLANNER_STEER_P.periodic();
                CONTROLLER_1_PORT.periodic();

                yamlDataHolder.periodic();

        }

        public Command getAutonomousCommand() {
                for (Trajectory path : pathGroup) {
                        fieldSim.setTrajectory(path);
                }

                return swerveAutoBuilder.fullAuto(pathGroup);
        }

}
