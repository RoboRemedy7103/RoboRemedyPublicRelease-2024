// RobotPeriodic.java - Code for robot periodic code (all modes)
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class RobotPeriodic {
    private Electronics e;
    private RobotState robotState;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private Dashboard dash;

    private boolean checkConfigsSetCalled = false;

    public RobotPeriodic(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.robotState = robotState;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: RobotPeriodic.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: RobotPeriodic.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: RobotPeriodic.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: RobotPeriodic.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: RobotPeriodic.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: RobotPeriodic.dash is null");
        }
    }

    public void robotPeriodic() {
        // if (!turnEncodersInit) {
        //     turnEncodersInit = e.initSwerveTurnEncoders();
        // }

        oi.refreshJoysticks();

        e.sendLEDBuffer();
        e.robotPeriodic();
        if ((robotState.robotElapsedTime - e.lastMotorInitTime > 1) && !e.areAllConfigsSet()) {
            e.checkConfigsSet(false);
            e.lastMotorInitTime = robotState.robotElapsedTime;
        }

        if (oi.getFixSwerveButtonPressed()) {
            if (!checkConfigsSetCalled) {
                e.checkConfigsSet(true);
                checkConfigsSetCalled = true;
            }
        } else {
            checkConfigsSetCalled = false;
        }

        dash.dashboardPeriodic();
        if (DriverStation.isDSAttached()) {
            if (oi.getDriverResetGyroButtonPressed()) {
                e.setGyro(0);
                e.resetTilt();
            } else if (oi.getDriverReverseResetGyroButtonPressed()) {
                e.setGyro(180);
                e.resetTilt();
            }
        }
        act.logChanges();
    }

}