// Robot.java - Main robot code for 2025
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotEnums.*;

public class Robot extends TimedRobot {

    private final String projectName = "2025 Competition";
    private RoboLog rLog = new RoboLog();
    private Electronics e = new Electronics(true, rLog);
    private OI oi = new OI(rLog);
    private RobotState robotState = new RobotState(e, rLog);
    private Action act = new Action(e, rLog, oi, robotState);;
    private Dashboard dash = new Dashboard(e, rLog, act, oi, robotState);
    private AutoRobot auto = new AutoRobot(e, rLog, act, robotState, dash);
    private TeleopRobot teleop = new TeleopRobot(e, rLog, act, oi, robotState, dash);
    private TestRobot test = new TestRobot(e, rLog, act, oi, robotState, dash);
    private RobotPeriodic periodic = new RobotPeriodic(e, rLog, act, oi, robotState, dash);
    private DisabledRobot disabled = new DisabledRobot(e, rLog, act, oi, robotState, dash);
    private SimulationRobot simulation = null;
    private Timer autoTimer = new Timer();
    private boolean everFromAuto = false;

    //private boolean turnEncodersInit = false;

    /* Constructor: called once when robot code is started */
    Robot() {
        try {
            rLog.print(projectName, " Robot constructor start");
            e.assignAllSwerveEncoderValues();
            if (robotState.isSimulation) {
                simulation = new SimulationRobot(e, rLog, act, oi, robotState, dash);
            } else {
                simulation = null;
            }
            //turnEncodersInit = e.initSwerveTurnEncoders();
            LiveWindow.disableAllTelemetry(); // Improve performance
            SmartDashboard.updateValues(); // Improve performance
            e.setSwerveCoast();
            e.resetTilt();
            e.setGyro(0);
            oi.assignJoysticks();
            autoTimer.restart();
            try {
                Thread.sleep(4000);
            } catch (Exception e) {
            }
            System.gc();
            rLog.print(projectName, " Robot constructor complete");
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    // Called once whenever robot is disabled
    @Override
    public void disabledInit() {
        try {
            rLog.setRobotMode("DISA", "Disabled");
            disabled.disabledInit();
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called periodically while robot is disabled */
    @Override
    public void disabledPeriodic() {
        try {
            robotState.robotStatePeriodic(dash, simulation);
            oi.logButtonChanges();
            disabled.disabledPeriodic();
            test.robotPeriodic(); // called in disabled and test modes
            periodic.robotPeriodic(); // called in all four modes
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called once when autonomous is started */
    @Override
    public void autonomousInit() {
        try {
            rLog.setRobotMode("AUTO", "Autonomous");
            AutoProgram autoSelected = dash.geAutoProgram();
            auto.autonomousInit(autoSelected);
            everFromAuto = true;
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called periodically during autonomous */
    @Override
    public void autonomousPeriodic() {
        try {
            robotState.robotStatePeriodic(dash, simulation);
            oi.logButtonChanges();
            auto.autonomousPeriodic();
            autoTimer.restart();
            periodic.robotPeriodic(); // called in all four modes
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called once when operator control is started */
    @Override
    public void teleopInit() {
        try {
            rLog.setRobotMode("TELE", "Teleop");
            if (autoTimer.get() < 4 && everFromAuto) {
                rLog.print("From Auto");
                teleop.teleopInit(true);
            } else {
                teleop.teleopInit(false);
            }
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called periodically during operator control */
    @Override
    public void teleopPeriodic() {
        try {
            robotState.robotStatePeriodic(dash, simulation);
            oi.logButtonChanges();
            teleop.teleopPeriodic();
            periodic.robotPeriodic(); // called in all four modes
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called once when test mode is started */
    @Override
    public void testInit() {
        try {
            rLog.setRobotMode("TEST", "Test");
            test.testInit();
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    /* Called periodically during test mode */
    @Override
    public void testPeriodic() {
        try {
            robotState.robotStatePeriodic(dash, simulation);
            oi.logButtonChanges();
            test.robotPeriodic(); // called in disabled and test modes
            test.testPeriodic(); // called only in test mode
            periodic.robotPeriodic(); // called in all four modes
        } catch (Exception e) {
            rLog.printException(e);
            throw (e);
        }
    }

    // We do not use this method, but we need to include it
    // to prevent warning messages.
    @Override
    public void robotPeriodic() {}
    
    // We do not use this method, but we need to include it
    // to prevent warning messages.
    @Override
    public void simulationPeriodic() {}
}