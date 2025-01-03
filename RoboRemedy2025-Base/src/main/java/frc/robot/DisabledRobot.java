// DisabledRobot.java - Code for disabled mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class DisabledRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Dashboard dash;

    public DisabledRobot(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: DisabledRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: DisabledRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: DisabledRobot.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: DisabledRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: DisabledRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: DisabledRobot.dash is null");
        }
    }

    public void disabledInit() {
        e.stopSwerveMotors();
    }

    public void disabledPeriodic() {
        if (robotState.robotElapsedTime > 23) {
            if (!robotState.isMainAprilTagCameraAttached || !robotState.isGamePieceCameraAttached) {
                e.setLED(LEDregion.RobotStatus, LEDcolor.Blue);
                e.setLED(LEDregion.BellyPan, LEDcolor.DimRed);
            } else if (!robotState.isFLDriveGood || !robotState.isFLTurnGood
                    || !robotState.isFRDriveGood || !robotState.isFRTurnGood
                    || !robotState.isBLDriveGood || !robotState.isBLTurnGood
                    || !robotState.isBRDriveGood || !robotState.isBRTurnGood) {
                e.setLED(LEDregion.RobotStatus, LEDcolor.Red);
                e.setLED(LEDregion.BellyPan, LEDcolor.DimRed);
            } else if (!robotState.isFLCancoderGood || !robotState.isFRCancoderGood 
                    || !robotState.isBLCancoderGood || !robotState.isBRCancoderGood) {
                e.setLED(LEDregion.RobotStatus, LEDcolor.White);
                e.setLED(LEDregion.BellyPan, LEDcolor.DimRed);
            } else if (!robotState.isBatteryGood) {
                e.setLED(LEDregion.RobotStatus, LEDcolor.Yellow);
                e.setLED(LEDregion.BellyPan, LEDcolor.DimRed);
            } else {
                e.setLED(LEDregion.RobotStatus, LEDcolor.Black);
                e.setLED(LEDregion.BellyPan, LEDcolor.DimWhite);
            }
        } else {
            e.setLED(LEDregion.RobotStatus, LEDcolor.Black);
            e.setLED(LEDregion.BellyPan, LEDcolor.Black);
        }
        robotState.doSwerveTurnEncodersMatch = e.doSwerveEncodersMatch();

        if (DriverStation.isDSAttached() == false) {
            e.setLED(LEDregion.Alliance, LEDcolor.DimWhite);
        } else if (robotState.isRedAlliance) {
            e.setLED(LEDregion.Alliance, LEDcolor.DimRed);
        } else {
            e.setLED(LEDregion.Alliance, LEDcolor.DimBlue);
        }

        oi.setDriverLeftRumble(0.00);
        oi.setDriverRightRumble(0.00);
        oi.setOperatorLeftRumble(0.00);
        oi.setOperatorRightRumble(0.00);
    }
}