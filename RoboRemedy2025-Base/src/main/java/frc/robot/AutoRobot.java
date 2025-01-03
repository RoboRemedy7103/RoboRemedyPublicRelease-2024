// AutoRobot.java - Code for autonomous programs
package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class AutoRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private Dashboard dash;
    private RobotState robotState;

    private int stepNumber;
    private Timer autoTimer = new Timer();
    private boolean isRedAlliance;
    private AutoProgram autoSelection;
    private AutoStartPosition autoPosition;

    private static final double MAX_ACCEL = 140.0;

    public AutoRobot(Electronics e, RoboLog rLog, Action act, RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: AutoRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: AutoRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: AutoRobot.act is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: AutoRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: AutoRobot.dash is null");
        }
    }

    public void autonomousInit(AutoProgram autoSelected) {
        this.isRedAlliance = robotState.isRedAlliance;
        this.autoSelection = autoSelected;
        this.autoPosition = dash.getAutoStart();
        rLog.print("Start Autonomous Program:" + autoSelected
            + ", Position:" + autoPosition
            + ", Alliance:" + (isRedAlliance ? "Red" : "Blue")
            + ", Battery:" + RobotMath.round1(robotState.batteryVoltage));
        e.setSwerveBrake();
        e.resetTilt();
        act.resetAction();
        setStepNumber(1);
        e.stopAllMotors();
        Pose2d pose = getRobotAutoPose(autoSelected, autoPosition);
        e.setGyro(pose.getRotation().getDegrees());
    }

    public void autonomousPeriodic() {
        if (autoSelection == AutoProgram.JustAlign) {
            e.alignSwerveMotorsForward();
        } else if (autoSelection == AutoProgram.DriveOut) {
            DriveOut();
        } else if (autoSelection == AutoProgram.SimulationProject) {
            SimulationProject();
        } else if (autoSelection == AutoProgram.None) {
            e.stopAllMotors();
        }

        if (stepNumber % 4 == 0) {
            e.setLED(LEDregion.AutoStep, LEDcolor.Yellow);
        } else if (stepNumber % 4 == 1) {
            e.setLED(LEDregion.AutoStep, LEDcolor.Green);
        } else if (stepNumber % 4 == 2) {
            e.setLED(LEDregion.AutoStep, LEDcolor.White);
        } else if (stepNumber % 4 == 3) {
            e.setLED(LEDregion.AutoStep, LEDcolor.Purple);
        }
    }

    void setStepNumber(int number) {
        stepNumber = number;
        act.resetAction();
        autoTimer.restart();
        rLog.print("New Auto Step Number:" + stepNumber + ", Gyro:" + robotState.gyroAngleRounded);
    }

    void setNextStepNumber() {
        setStepNumber(stepNumber + 1);
    }

    public Pose2d getRobotAutoPose(AutoProgram autoProgram, AutoStartPosition autoStartPosition) {
        double x = 0;
        double y = 0;
        double angle = 0;
        switch (autoProgram) {
            case JustAlign:
                angle = 0;
                break;
            case DriveOut:
                angle = 0;
                break;
            case SimulationProject:
                angle = 0;
            case None:
                angle = 0;
                break;
            default:
                angle = 0;
                break;
    }
        return new Pose2d(x, y, Rotation2d.fromDegrees(angle));
    }

    void DriveOut() {
        e.stopPivotMotor();
        switch (stepNumber) {
            case 1: // Drive Out: Drive forward 36 inches
                if (act.driveStraightWithFacing(0.0, 40.0,
                    0.0, MAX_ACCEL,
                    36.0, 0.0)) {
                    setNextStepNumber();
                }
                break;
            case 2:// Drive Out: Stop
                e.stopSwerveMotors();
                break;
        }
    }

    void SimulationProject() {
        e.stopPivotMotor();
        switch (stepNumber) {
            case 1: // Simulation Project: Drive forward
                if (act.driveStraightWithFacing(0, 30,
                    0, MAX_ACCEL, 45, 15)) {
                    setNextStepNumber();
                }
                break;
            case 2: // Simulation Project: Drive right
                if (act.driveStraightWithFacing(90, 30,
                    90, MAX_ACCEL, 45, 15)) {
                    setNextStepNumber();
                }
                break;
            case 3: // Simulation Project: Drive forward
                if (act.driveStraightWithFacing(0, 30,
                    180, MAX_ACCEL, 45, 15)) {
                    setNextStepNumber();
                }
                break;
            case 4: // Simulation Project: Stop
                e.stopAllMotors();
                break;
        }
    }
}