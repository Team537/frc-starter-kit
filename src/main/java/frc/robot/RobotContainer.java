package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.config.YAMLDataHolder;
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
    private XboxController controller = new XboxController(0);
    private YAMLDataHolder yamlDataHolder;
    private SwerveAutoBuilder swerveAutoBuilder;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));

    private LoggedTunableValue PATH_PLANNER_DRIVE_P = new LoggedTunableValue("PATH_PLANNER_DRIVE_P");
    private LoggedTunableValue PATH_PLANNER_DRIVE_I = new LoggedTunableValue("PATH_PLANNER_DRIVE_I");
    private LoggedTunableValue PATH_PLANNER_DRIVE_D = new LoggedTunableValue("PATH_PLANNER_DRIVE_D");
    private LoggedTunableValue PATH_PLANNER_STEER_P = new LoggedTunableValue("PATH_PLANNER_STEER_P");
    private LoggedTunableValue PATH_PLANNER_STEER_I = new LoggedTunableValue("PATH_PLANNER_STEER_I");
    private LoggedTunableValue PATH_PLANNER_STEER_D = new LoggedTunableValue("PATH_PLANNER_STEER_D");

    HashMap<String, Command> eventMap = new HashMap<>();

    public RobotContainer() {

        yamlDataHolder = YAMLDataHolder.getInstance();
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
                        () -> controller.getRightX(), true)

        );

    }

    public void onDisable() {
        swerve.stop();
        yamlDataHolder.saveData();

    }

    public void periodic() {
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

        PATH_PLANNER_DRIVE_D.periodic();
        PATH_PLANNER_DRIVE_I.periodic();
        PATH_PLANNER_DRIVE_P.periodic();
        PATH_PLANNER_STEER_D.periodic();
        PATH_PLANNER_STEER_I.periodic();
        PATH_PLANNER_STEER_P.periodic();

    }

    public Command getAutonomousCommand() {
        return swerveAutoBuilder.fullAuto(pathGroup);
    }

}
