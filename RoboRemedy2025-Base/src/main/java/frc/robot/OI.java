// OI.java - Operator Interface. Reads values from joysticks and buttons
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.hal.*;

public class OI {
    private RoboLog rLog;
    private boolean lastCancel = false;
    private boolean lastAlignWheels = false;
    private boolean lastLockWheels = false;
    private boolean lastDriverResetGyro = false;
    private boolean lastDriverReverseResetGyro = false;
    private boolean lastDriveFast = false;
    private boolean lastDriveSlow = false;
    private boolean lastDriverD2 = false;
    private boolean lastAutoTeleopStart = false;
    private boolean lastAimPivot = false;
    private boolean lastFastPivot = false;
    private boolean lastSlowPivot = false;
    private boolean lastFixSwerve = false;

    private short leftRumble = 0;
    private short rightRumble = 0;

    private String preferredDriverName = "Controller (Xbox One For Windows)";
    private int preferredDriverType = 1;
    private boolean driverAssigned = false;
    private String driverEligible = "Driver Eligible";
    private String activeDriver = "Active Driver";

    private String preferredOperatorName = "Controller (Xbox360 Controller for Windows)";
    private int preferredOperatorType = 1;
    private boolean operatorAssigned = false;
    private String operatorEligible = "Operator Eligible";
    private String activeOperator = "Active Operator";

    public OIJoystick[] Joysticks = {new OIJoystick(0), new OIJoystick(1), new OIJoystick(2), 
            new OIJoystick(3), new OIJoystick(4), new OIJoystick(5)}; //Likely private in the future

    private OIJoystick driverJoystick = null;
    private OIJoystick operatorJoystick = null;

    private String lastDriverName = "";
    private String lastOperatorName = "";

    private boolean isPrinted = false;
    private boolean driverDifferent = false;
    private boolean operatorDifferent = false;

    OI(RoboLog rLog) {
        this.rLog = rLog;

        if (this.rLog == null) {
            System.out.println("Warning: TeleopRobot.rLog is null");
        }
    }

    public void changePreferredDriverName(String newName) {
        preferredDriverName = newName;
    }

    public void changePreferredOperatorName(String newName) {
        preferredOperatorName = newName;
    }
    
    public void assignJoysticks() {
        int eligibleCount = 0;
        driverAssigned = false;
        operatorAssigned = false;
        driverJoystick = null;
        operatorJoystick = null;

        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                stick.setJoystickStatus("Null");
            }
        }

        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                if (stick.getJoystickName().equals(preferredDriverName)) {
                    stick.setJoystickStatus(driverEligible);
                    eligibleCount += 1;
                }
            }
        }

        if (eligibleCount < 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickType() == preferredDriverType) {
                        stick.setJoystickStatus(driverEligible);
                        eligibleCount += 1;
                    }
                }
            }
        }

        if (eligibleCount > 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickStatus().equals(driverEligible)) {
                        if (stick.getJoystickType() != preferredDriverType) {
                            stick.setJoystickStatus("Null");
                            eligibleCount -= 1;
                        }
                    }
                }
            }
        }

        if (eligibleCount < 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickType() != 0) {
                        stick.setJoystickStatus(driverEligible);
                        eligibleCount += 1;
                    }
                }
            }
        }

        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                if (!driverAssigned && stick.getJoystickStatus().equals(driverEligible)) {
                    stick.setJoystickStatus(activeDriver);
                    driverJoystick = stick;
                    driverAssigned = true;
                }
            }
        }

        eligibleCount = 0;

        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                if (stick.getJoystickName().equals(preferredOperatorName) && 
                        !stick.getJoystickStatus().equals(activeDriver)) {
                    stick.setJoystickStatus(operatorEligible);
                    eligibleCount += 1;
                }
            }
        }

        if (eligibleCount < 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickType() != preferredOperatorType) {
                        stick.setJoystickStatus(operatorEligible);
                        eligibleCount += 1;
                    }
                }
            }
        }

        if (eligibleCount > 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickStatus().equals(operatorEligible)) {
                        if (stick.getJoystickType() != preferredOperatorType) {
                            stick.setJoystickStatus("Null");
                            eligibleCount -= 1;
                        }
                    }
                }
            }
        }

        if (eligibleCount < 1) {
            for (OIJoystick stick : Joysticks) {
                if (stick != null) {
                    if (stick.getJoystickType() != 0 && stick.getJoystickStatus() != activeDriver) {
                        stick.setJoystickStatus(operatorEligible);
                        eligibleCount += 1;
                    }
                }
            }
        }

        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                if (!operatorAssigned && stick.getJoystickStatus().equals(operatorEligible)) {
                    stick.setJoystickStatus(activeOperator);
                    operatorJoystick = stick;
                    operatorAssigned = true;
                }
            }
        }
    }

    public boolean isDriverJoystickConnected() {
        if (driverJoystick == null)
            return false;
        return DriverStation.isJoystickConnected(driverJoystick.getJoystickPort());
    }

    public boolean isDriverTestButtonPressed() {
        return isJoystickButtonPressed(driverJoystick, 1);
    }

    public boolean isOperatorJoystickConnected() {
        if (operatorJoystick == null)
            return false;
        return DriverStation.isJoystickConnected(operatorJoystick.getJoystickPort());
    }

    public boolean isOperatorTestButtonPressed() {
        if (operatorJoystick == null)
            return false;
        if (!driverJoystick.isConnected())
            return false;
        return DriverStation.getStickButton(operatorJoystick.getJoystickPort(), 1);
    }

    public void refreshJoysticks() {
        if (driverJoystick != null)
            lastDriverName = driverJoystick.getJoystickName();
        else
            lastDriverName = "NULL";
        if (operatorJoystick != null)
            lastOperatorName = operatorJoystick.getJoystickName();
        else
            lastOperatorName = "NULL";
        for (OIJoystick stick : Joysticks) {
            if (stick != null) {
                stick.refreshJoystickInfo();
            }
        }
        if (driverJoystick == null || operatorJoystick == null) {
            assignJoysticks();
            if (!isPrinted) {
                if (driverJoystick != null) {
                    if (driverJoystick.isConnected()) {
                        rLog.print("New Driver Joystick: " + driverJoystick.getJoystickName() + " at " 
                            + driverJoystick.getJoystickPort());
                    } else {
                        rLog.print("Driver Joystick not connected");
                    }
                } else {
                    rLog.print("Driver Joystick null");
                }
                if (operatorJoystick != null) {
                    if (operatorJoystick.isConnected()) {
                        rLog.print("New Operator Joystick: " + operatorJoystick.getJoystickName() + " at " 
                            + operatorJoystick.getJoystickPort());
                    } else {
                        rLog.print("Operator Joystick not connected");
                    }
                } else {
                    rLog.print("Operator Joystick null");
                }
                isPrinted = true;
            }
        } else if (driverJoystick.getJoystickName() != lastDriverName || 
                operatorJoystick.getJoystickName() != lastOperatorName) {
            assignJoysticks();
            if (!isPrinted) {
                if (driverJoystick != null) {
                    if (driverJoystick.isConnected()) {
                        rLog.print("New Driver Joystick: " + driverJoystick.getJoystickName() + " at " 
                            + driverJoystick.getJoystickPort());
                    } else {
                        rLog.print("Driver Joystick not connected");
                    }
                } else {
                    rLog.print("Driver Joystick null");
                }
                if (operatorJoystick != null) {
                    if (operatorJoystick.isConnected()) {
                        rLog.print("New Operator Joystick: " + operatorJoystick.getJoystickName() + " at " 
                            + operatorJoystick.getJoystickPort());
                    } else {
                        rLog.print("Operator Joystick not connected");
                    }
                } else {
                    rLog.print("Operator Joystick null");
                }
                isPrinted = true;
            }
        }
        driverDifferent = false;
        operatorDifferent = false;
        if (driverJoystick == null) {
            if (!lastDriverName.equals("NULL"))
                driverDifferent = true;
        } else {
            if (!driverJoystick.getJoystickName().equals(lastDriverName))
                driverDifferent = true;
        }
        if (operatorJoystick == null) {
            if (!lastOperatorName.equals("NULL"))
                operatorDifferent = true;
        } else {
            if (!operatorJoystick.getJoystickName().equals(lastDriverName))
                operatorDifferent = true;
        }
        if (driverDifferent || operatorDifferent) {
            isPrinted = false;
        }
    }

    public boolean isJoystickButtonPressed(int stick, int button) {
        if (DriverStation.isJoystickConnected(stick)) {
            return DriverStation.getStickButton(stick, button);
        } else {
            return false;
        }
    }

    private boolean isJoystickButtonPressed(OIJoystick stick, int button) {
        if (stick == null)
            return false;
        if (!stick.isConnected())
            return false;
        return DriverStation.getStickButton(stick.getJoystickPort(), button);
    }

    public boolean isJoystickPovPressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            return (DriverStation.getStickPOV(stick, 0) == value);
        } else {
            return false;
        }
    }

    private boolean isJoystickTriggerPressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            if (DriverStation.getStickAxis(stick, value) > 0.5) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean isJoystickTriggerNegativePressed(int stick, int value) {
        if (DriverStation.isJoystickConnected(stick)) {
            if (DriverStation.getStickAxis(stick, value) < -0.5) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public double getJoystickAxis(int stick, int axis) {
        if (DriverStation.isJoystickConnected(stick) &&
            axis < DriverStation.getStickAxisCount(stick)) {
            return DriverStation.getStickAxis(stick, axis);
        } else {
            return 0;
        }
    }

    public int getPOV(int stick) {
        if (DriverStation.isJoystickConnected(stick)) {
            return (DriverStation.getStickPOV(stick, 0));
        } else {
            return 0; 
        }
    }

    public double getMagnitude(int stick, int axis1, int axis2) {
        return Math.sqrt(Math.pow(getJoystickAxis(stick, axis1), 2) + Math.pow(getJoystickAxis(stick, axis2), 2));
    }

    public double getDirectionDegrees(int stick, int axis1, int axis2) {
        double x = getJoystickAxis(stick, axis1);
        double y = getJoystickAxis(stick, axis2);
        double angleRad = Math.atan2(y, x);
        double angleDeg = Math.toDegrees(angleRad);
        double finalAng = angleDeg + 90;
        return finalAng;
    }

    // rumble stuff

    public void setDriverLeftRumble(double leftRumble) {
        this.leftRumble = (short)(RobotMath.minMax(0, 1, leftRumble) * 65535);
        DriverStationJNI.setJoystickOutputs((byte)0, 0, this.leftRumble, this.rightRumble);
    }

    public void setOperatorLeftRumble(double leftRumble) {
        this.leftRumble = (short)(RobotMath.minMax(0, 1, leftRumble) * 65535);
        DriverStationJNI.setJoystickOutputs((byte)1, 0, this.leftRumble, this.rightRumble);
    }
    
    public void setDriverRightRumble(double rightRumble) {
        this.rightRumble = (short)(RobotMath.minMax(0, 1, rightRumble) * 65535);
        DriverStationJNI.setJoystickOutputs((byte)0, 0, this.leftRumble, this.rightRumble);
    }

    public void setOperatorRightRumble(double rightRumble) {
        this.rightRumble = (short)(RobotMath.minMax(0, 1, rightRumble) * 65535);
        DriverStationJNI.setJoystickOutputs((byte)1, 0, this.leftRumble, this.rightRumble);
    }

    // driver joysticks

    public double getDriveMagnitude() {
        return getMagnitude(0, 0, 1);
    }

    public double getDriveDirectionDegrees() {
        return getDirectionDegrees(0, 0, 1);
    }

    public double getFacingJoystickMagnitude() {
        return getMagnitude(0, 4, 5);
    }

    public double getFacingJoystickDegrees() {
        return getDirectionDegrees(0, 4, 5);
    } 

    // driver POV buttons

    public boolean getDriverReverseResetGyroButtonPressed() {
        return isJoystickPovPressed(0, 0);
    }

    public boolean getDriverResetGyroButtonPressed() {
        return isJoystickPovPressed(0, 90);
    }

    public boolean getAlignWheelsButtonPressed() {
        return isJoystickPovPressed(0, 180);
    }

    public boolean getLockButtonPressed() {
        return isJoystickPovPressed(0, 270);
    }

    // driver buttons

    public boolean getDriverButtonPressed_D1() {
        return isJoystickButtonPressed(1, 1);
    }

    public boolean getDriverButtonPressed_D2() {
        return isJoystickButtonPressed(0, 2);
    }

    public boolean getAutoTeleopStartPressed_D3() {
        return isJoystickButtonPressed(0, 3);
    }

    public boolean getDriveSlowButtonPressed() {
        return isJoystickButtonPressed(0, 6);
    }

    public boolean getDriveFastButtonPressed() {
        return isJoystickTriggerPressed(0, 3);
    }

    public boolean getCancelButtonPressed_D4() {
        return isJoystickButtonPressed(0, 4);
    }

    // operator joysticks

    public double getPivotSpeed() {
        return - getJoystickAxis(1,1);
    }

    // operator POV buttons

    // operator buttons

    public boolean getSlowPivotButtonPressed() {
        return isJoystickButtonPressed(1, 6);
    }

    public boolean getFastPivotButtonPressed() {
        return isJoystickTriggerPressed(1, 3);
    }

    public boolean getAimPivotButtonPressed_O1() {
        return isJoystickButtonPressed(1, 1);
    } 

    public boolean getFixSwerveButtonPressed() {
        return isJoystickButtonPressed(1, 7);
    } 

    public void logButtonChanges() {
        boolean cancel = getCancelButtonPressed_D4();
        boolean alignWheels = getAlignWheelsButtonPressed();
        boolean lockWheels = getLockButtonPressed();
        boolean driverResetGyro = getDriverResetGyroButtonPressed();   
        boolean driverReverseResetGyro = getDriverReverseResetGyroButtonPressed();
        boolean driveSlow = getDriveSlowButtonPressed();
        boolean driveFast = getDriveFastButtonPressed();
        boolean driverD2 = getDriverButtonPressed_D2();
        boolean autoTeleopStart = getAutoTeleopStartPressed_D3();
        boolean slowPivot = getSlowPivotButtonPressed();
        boolean fastPivot = getFastPivotButtonPressed();
        boolean aimPivot = getAimPivotButtonPressed_O1();
        boolean fixSwerve = getFixSwerveButtonPressed();

        if (cancel != lastCancel) {
            if(cancel)
                rLog.print("Cancel Pressed");
            else
                rLog.print("Cancel Released");
            lastCancel = cancel;
        }

        if (alignWheels != lastAlignWheels) {
            if (alignWheels)
                rLog.print("Align Wheels Pressed");
            else
                rLog.print("Align Wheels Released");
            lastAlignWheels = alignWheels;
        }
    
        if (lockWheels != lastLockWheels) {
            if (lockWheels)
                rLog.print("Lock Wheels Pressed");
            else
                rLog.print("Lock Wheels Released");
            lastLockWheels = lockWheels;
        }
    
        if (driverResetGyro != lastDriverResetGyro) {
            if (driverResetGyro)
                rLog.print("Driver Reset Gyro Pressed");
            else
                rLog.print("Driver Reset Gyro Released");
            lastDriverResetGyro = driverResetGyro;
        }
    
        if (driverReverseResetGyro != lastDriverReverseResetGyro) {
            if (driverReverseResetGyro)
                rLog.print("Driver Reverse Reset Gyro Pressed");
            else
                rLog.print("Driver Reverse Reset Gyro Released");
            lastDriverReverseResetGyro = driverReverseResetGyro;
        }
    
        if (driveSlow != lastDriveSlow) {
            if (driveSlow)
                rLog.print("Drive Slow Pressed");
            else
                rLog.print("Drive Slow Released");
            lastDriveSlow = driveSlow;
        }

        if (driveFast != lastDriveFast) {
            if (driveFast)
                rLog.print("Drive Fast Pressed");
            else
                rLog.print("Drive Fast Released");
            lastDriveFast = driveFast;
        }

        if (driverD2 != lastDriverD2) {
            if (driverD2)
                rLog.print("Driver D2 Pressed");
            else
                rLog.print("Driver D2 Released");
            lastDriverD2 = driverD2;
        }

        if (autoTeleopStart != lastAutoTeleopStart) {
            if (autoTeleopStart)
                rLog.print("Auto Teleop Start Pressed");
            else
                rLog.print("Auto Teleop Start Released");
            lastAutoTeleopStart = autoTeleopStart;
        }

        if (slowPivot != lastSlowPivot) {
            if (slowPivot)
                rLog.print("Slow Pivot Pressed");
            else
                rLog.print("Slow Pivot Released");
            lastSlowPivot = slowPivot;
        }

        if (fastPivot != lastFastPivot) {
            if (fastPivot)
                rLog.print("Fast Pivot Pressed");
            else
                rLog.print("Fast Pivot Released");
            lastFastPivot = fastPivot;
        }

        if (aimPivot != lastAimPivot) {
            if (aimPivot)
                rLog.print("Aim Pivot Pressed");
            else
                rLog.print("Aim Pivot Released");
            lastAimPivot = aimPivot;
        }

        if (fixSwerve != lastFixSwerve) {
            if (fixSwerve)
                rLog.print("Fix Swerve Pressed");
            else
                rLog.print("Fix Swerve Released");
            lastFixSwerve = fixSwerve;
        }
    }
}
