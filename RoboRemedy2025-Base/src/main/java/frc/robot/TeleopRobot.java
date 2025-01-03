// TeleopRobot.java - Code for teleop mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class TeleopRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private OI oi;
    private RobotState robotState;
    private Dashboard dash;

    private boolean isTeleopProgramRunning = false;
    private TeleopProgram whichTeleopProgram = TeleopProgram.None;
    private int teleopStepNumber = 0;

    private double lastDriveDirection = 0.0;
    private double lastFactingAngle = 0.0;

    public TeleopRobot(Electronics e, RoboLog rLog, Action act, OI oi,
            RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: TeleopRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: TeleopRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: TeleopRobot.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: TeleopRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: TeleopRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: TeleopRobot.dash is null");
        }
    }

    public void teleopInit(boolean fromAuto) {
        e.setSwerveBrake();
        act.resetAction();
        lastDriveDirection = 0.0;
        teleopStepNumber = 1;
        isTeleopProgramRunning = false;
    }

    void setTeleopStepNumber(int number) {
        teleopStepNumber = number;
        act.resetAction();
        rLog.print("New Teleop Step: " + teleopStepNumber
            + ", Program: " + whichTeleopProgram);
    }

    public void teleopPeriodic() {
        double joyMag = Math.pow(oi.getDriveMagnitude(), 2);
        double driveSpeed;
        double driveDirection;
        double facing;
        boolean isStopped = false;

        // Need to start an auto-teleop program?
        if (oi.getAutoTeleopStartPressed_D3()) {
            if (!isTeleopProgramRunning || whichTeleopProgram != TeleopProgram.TeleopSample) {
                whichTeleopProgram = TeleopProgram.TeleopSample;
                isTeleopProgramRunning = true;
                setTeleopStepNumber(1);
            }
        } else {
            isTeleopProgramRunning = false;
            whichTeleopProgram = TeleopProgram.None;
        }

        if (isTeleopProgramRunning) {
            switch (whichTeleopProgram) {
                case TeleopSample:
                    switch (teleopStepNumber) {
                        case 1: // TeleopSample: Drive 5 inches
                            if (act.driveStraightWithFacing(0, 5, 0, 5, 5, 0)) {
                                setTeleopStepNumber(2);
                            }
                            break;
                        case 5: // TeleopSample: Done
                            e.stopAllMotors();;
                            break;
                    }
                    break;

                default:
                    isTeleopProgramRunning = false;
                    break;
            }
        } else {
            whichTeleopProgram = TeleopProgram.None;
        }

        // Teleop driving, driver can't control robot in teleop program mode

        if (isTeleopProgramRunning == false) {
            if (oi.getDriverResetGyroButtonPressed() || oi.getDriverReverseResetGyroButtonPressed()) {
                facing = robotState.gyroAngle;
            } else {
                facing = lastFactingAngle;
            }

            if (oi.getDriveFastButtonPressed() || oi.getDriveSlowButtonPressed()) {
                if (joyMag < 0.1) {
                    driveSpeed = 0;
                } else {
                     // We used speeds of 190 / 60 by end of 2024 season
                    double maxSpeed = (oi.getDriveFastButtonPressed() ? 100 : 40);
                    driveSpeed = (joyMag - 0.1) * (maxSpeed / 0.9);
                }
                driveDirection = oi.getDriveDirectionDegrees();
                double facingMagnitude = oi.getFacingJoystickMagnitude();
                double facingDegrees = oi.getFacingJoystickDegrees();

                if (facingMagnitude >= .9) {
                    facing = facingDegrees;
                }

                isStopped = false;
            } else {
                driveSpeed = 0.0;
                driveDirection = lastDriveDirection;
                isStopped = true;
            }

            lastDriveDirection = driveDirection;

            if (!isStopped) {
                e.assignRobotMotionAndHeadingField(driveDirection, driveSpeed, facing);
            } else if (oi.getAlignWheelsButtonPressed()) {
                e.alignSwerveMotorsForward();
            } else if (oi.getLockButtonPressed()) {
                e.lockWheels();
            } else {
                e.stopSwerveMotors();
            }

            // Control Pivot
            if (oi.getSlowPivotButtonPressed() || oi.getFastPivotButtonPressed()) {
                double maxSpeed = (oi.getSlowPivotButtonPressed() ? 0.03 : 0.2);
                double pivotSpeed = (oi.getPivotSpeed() * maxSpeed);
                if (pivotSpeed > 0 && robotState.pivotAngle > RobotState.MAX_PIVOT_ANGLE) {
                    pivotSpeed = 0;
                } else if (pivotSpeed < 0 && robotState.pivotAngle < RobotState.MIN_PIVOT_ANGLE) {
                    pivotSpeed = 0;
                }
                e.setPivotMotorPercent(pivotSpeed);
            } else if (oi.getAimPivotButtonPressed_O1()) {
                e.movePivotToAngle(RobotState.PIVOT_AIM_ANGLE);
            } else {
                e.stopPivotMotor();
            }
        }

        // Display the countdown
        double timeRemaining = DriverStation.getMatchTime();

        if (timeRemaining < 20 && timeRemaining > 2) {
            oi.setDriverLeftRumble(.75 - (timeRemaining / 40.0));
            oi.setDriverRightRumble(.75 - (timeRemaining / 40.0));
            oi.setOperatorLeftRumble(.75 - (timeRemaining / 40.0));
            oi.setOperatorRightRumble(.75 - (timeRemaining / 40.0));
        } else {
            oi.setDriverLeftRumble(0);
            oi.setDriverRightRumble(0);
            oi.setOperatorLeftRumble(0);
            oi.setOperatorRightRumble(0);
        }

        e.setLED(LEDregion.RegionAll, LEDcolor.Black);

        if (timeRemaining > 0 && timeRemaining <= 30.0) {
            e.setLEDsForCountdown(DriverStation.getMatchTime());
        }
    }
}