package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.ModuleIO;
import frc.robot.subsystems.Swerve.ModuleIOFalcon500;
import frc.robot.subsystems.Swerve.ModuleIOSim;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.utils.ModulePosition;

public class RobotContainer {
    private Swerve swerve;
    private Mode mode = Mode.SIM;
    private XboxController controller = new XboxController(0);
    

    

    public RobotContainer() {
        switch(mode) {
            case REAL:
                swerve = new Swerve( new GyroIOPigeon2(), 
                new ModuleIOFalcon500(ModulePosition.FRONT_LEFT), 
                new ModuleIOFalcon500(ModulePosition.FRONT_RIGHT), 
                new ModuleIOFalcon500(ModulePosition.BACK_LEFT), 
                new ModuleIOFalcon500(ModulePosition.BACK_RIGHT));
                break;
            case SIM:
                swerve = new Swerve(new GyroIOPigeon2() , 
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
                break;
            default:
                swerve = new Swerve( new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
            
                break;
        }

        swerve.setDefaultCommand(

        new RunCommand(() ->  swerve.drive(-controller.getLeftY(), controller.getLeftX(), -controller.getRightX(), true), swerve)

        );

    }

    public void onDisable(){
        swerve.stop();
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
