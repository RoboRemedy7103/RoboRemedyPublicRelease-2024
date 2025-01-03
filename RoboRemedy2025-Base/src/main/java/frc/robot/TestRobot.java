// TestRobot.java - Code for test mode
package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.RobotEnums.*;

public class TestRobot {
    private Electronics e;
    private RoboLog rLog;
    private Action act;
    private Timer testTimer = new Timer();
    private Dashboard dash;
    private RobotState robotState;
    private OI oi;
    private boolean isButtonPressed = false;
    private boolean lastButtonPressed = false;
    private boolean lastZeroButtonPressed = false;

    public TestRobot(Electronics e, RoboLog rLog, Action act, OI oi,
        RobotState robotState, Dashboard dash) {
        this.e = e;
        this.rLog = rLog;
        this.act = act;
        this.oi = oi;
        this.robotState = robotState;
        this.dash = dash;

        if (this.rLog == null) {
            System.out.println("Warning: TestRobot.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: TestRobot.e is null");
        }
        if (this.act == null) {
            rLog.print("Warning: TestRobot.act is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: TestRobot.oi is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: TestRobot.robotState is null");
        }
        if (this.dash == null) {
            rLog.print("Warning: TestRobot.dash is null");
        }
    }

    public void testInit() {
        act.resetAction();
        testTimer.restart();
        rLog.print("Start Test Program: " + dash.getTestMode());
        e.stopAllMotors();
        lastButtonPressed = false;
    }

    public void robotPeriodic() {
        // This code runs in disabled and test modes
        TestMode currentTestMode = dash.getTestMode();

        if (currentTestMode == TestMode.QuickTest) {
            dash.setTestInstructions("");
        } else if (currentTestMode == TestMode.SwerveTest) {
            dash.setTestInstructions("A=Drv,B=Trn,X=Drv%,Y=Trn%,LB=0°,RB=90°,Back=FaceJoy");
        } else if (currentTestMode == TestMode.ResetPivotEncoder) {
            dash.setTestInstructions("A=Reset Pivot");
        } else if (currentTestMode == TestMode.TestMotors) {
            dash.setTestInstructions("A=I+,B=I-,X=T+,Y=T-,LB=P+,RB=P-,Back=LS,Menu=RS");
        } else if (currentTestMode == TestMode.SwerveDrive) {
            dash.setTestInstructions("A=in./sec 1.5, B=in./sec 3.7");
        } else if (currentTestMode == TestMode.TestFacing) {
            dash.setTestInstructions("Facing: A=-1,B=-1.5,X=-3,Y=-12");
        } else if (currentTestMode == TestMode.ResetSwerveEncoders) {
            dash.setTestInstructions("Gears to Right. A=Reset, B=Assign All");
        } else if (currentTestMode == TestMode.TestLEDRegions) {
            dash.setTestInstructions(
                    "A=Belly, B=LS, X=RS, Y=LT, LB=RT, RB=Robo Status");
        } else {
            dash.setTestInstructions("");
        }
    }

    public void testPeriodic() {
        TestMode currentTestMode = dash.getTestMode();
        if (currentTestMode == TestMode.QuickTest)
            quickTest();
        else if (currentTestMode == TestMode.SwerveTest)
            testSwerve();
        else if (currentTestMode == TestMode.SwerveDrive)
            testSwerveDrive();
        else if (currentTestMode == TestMode.TestFacing)
            testFacing();
        else if (currentTestMode == TestMode.TestMotors)
            testMotors();
        else if (currentTestMode == TestMode.TestMotorsWJoystick)
            TestMotorsWJoystick();
        else if (currentTestMode == TestMode.TuneMotor)
            testTuneMotor();
        else if (currentTestMode == TestMode.TestLEDRegions)
            TestLEDregions();
        else if (currentTestMode == TestMode.TestLEDColors)
            TestLEDColors();
        else if (currentTestMode == TestMode.ResetSwerveEncoders)
            ResetSwerveEncoder();
        else if (currentTestMode == TestMode.ResetPivotEncoder)
            ResetPivotEncoder();
        else if (currentTestMode == TestMode.SlowPivot)
            testSlowPivot();
        else if (currentTestMode == TestMode.AimPivot)
            testAimPivot();
        else if (currentTestMode == TestMode.TestCountdown)
            e.setLEDsForCountdown(30 - testTimer.get());
    }

    // Add testing subjects in here to make a quick run
    public void quickTest() {
        dash.setTestString("Enter Text Here");
    }

    public void testSwerve() {
        int module = 0;
        if (oi.isJoystickButtonPressed(0, 9)) {
            module = 1;
        } else if (oi.isJoystickPovPressed(0, 180)) {
            module = 2;
        } else if (oi.isJoystickPovPressed(0, 225)) {
            module = 3;
        }

        double pow = -1 * oi.getJoystickAxis(0, 1);
        double mag = oi.getMagnitude(0, 0, 1);
        double goalAng = oi.getDirectionDegrees(0, 0, 1);
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setDrivePercent(module, 0.2);
            e.setTurnPercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.setTurnPercent(module, 0.2);
            e.setDrivePercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            e.setDrivePercent(module, 0.8 * pow);
            e.setTurnPercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            e.setTurnPercent(module, 0.8 * pow);
            e.setDrivePercent(module, 0);
        } else if (oi.isJoystickButtonPressed(0, 5)) {
            e.setAllHeadings(0);
        } else if (oi.isJoystickButtonPressed(0, 6)) {
            e.setAllHeadings(90);
        } else if (oi.isJoystickButtonPressed(0, 7)) {
            if (mag > 0.3) {
                e.setAllHeadings(goalAng);
            }
        } else if (oi.isJoystickPovPressed(0, 0)) {
            e.setDriveSpeed(module, 4);
            rLog.print("Velocity, cmd = 4 act = " + e.getDriveVelocity(module) +
                    " out% = " + e.getDriveOutputPercentage(module));
        } else if (oi.isJoystickPovPressed(0, 45)) {
            e.setDriveSpeed(module, 12);
            rLog.print("Velocity, cmd = 12 act = " + e.getDriveVelocity(module) +
                    " out% = " + e.getDriveOutputPercentage(module));
        } else if (oi.isJoystickPovPressed(0, 90)) {
            double velocity = pow * 100.0;
            e.setDriveSpeed(module, velocity);
            rLog.print("Velocity, cmd = " + velocity + " act = " + e.getDriveVelocity(module) +
                    " out% = " + e.getDriveOutputPercentage(module));
        } else {
            e.stopSwerveMotors();
        }
        if (oi.isJoystickPovPressed(0, 270)) {
            rLog.print("Abs Encoders Front Left: " + e.getAbsoluteTurnEncoderPosition(0) + "Front Right: "
                    + e.getAbsoluteTurnEncoderPosition(1) + "Back Right: " + e.getAbsoluteTurnEncoderPosition(2)
                    + "Back Left: " + e.getAbsoluteTurnEncoderPosition(3));
        } else if (oi.isJoystickPovPressed(0, 315)) {
            rLog.print("non-ABS Encoedrs Front Left:" + e.getTurnEncoderPosition(0) + "Front Right: "
                    + e.getTurnEncoderPosition(1) + "Back Right: " + e.getTurnEncoderPosition(2)
                    + "Back Left: " + e.getTurnEncoderPosition(3));
        }
    }

    public void testSwerveDrive() {
        boolean isButtonPressed = false;
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.assignRobotMotionRobot(0, 1.5, 0);
            isButtonPressed = true;
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.assignRobotMotionRobot(0, 3.7, 0);
            isButtonPressed = true;
        } else {
            e.stopSwerveMotors();
        }

        if (isButtonPressed) {
            RobotMotor motor1 = e.getSwerveDriveMotor(1);
            RobotMotor motor3 = e.getSwerveDriveMotor(3);
            rLog.print("Speed1 = " + RobotMath.round2(motor1.getEncoderVelocity()) +
                    "Speed3 = " + RobotMath.round2(motor3.getEncoderVelocity()));
        }
    }

    public void testFacing() {
        if (oi.isJoystickButtonPressed(0, 1))
            e.assignRobotMotionAndHeadingField(0, 0, -1);
        else if (oi.isJoystickButtonPressed(0, 2))
            e.assignRobotMotionAndHeadingField(0, 0, -1.5);
        else if (oi.isJoystickButtonPressed(0, 3))
            e.assignRobotMotionAndHeadingField(0, 0, -3);
        else if (oi.isJoystickButtonPressed(0, 4))
            e.assignRobotMotionAndHeadingField(0, 0, -12);
        else
            e.stopSwerveMotors();
    }

    public void testMotors() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setSampleMotorPercent(0.3);
        } else {
            e.stopSampleMotor();
        }

        if (oi.isJoystickButtonPressed(0, 2)) {
            e.setPivotMotorPercent(0.3);
        } else {
            e.stopPivotMotor();
        }
    }

    public void TestMotorsWJoystick() {
        double joystick = oi.getJoystickAxis(0, 0);

        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setSampleMotorPercent(joystick);
        } else {
            e.stopSampleMotor();
        }

        if (oi.isJoystickButtonPressed(0, 2)) {
            e.setPivotMotorPercent(joystick);
        } else {
            e.stopPivotMotor();
        }
    }

    public void testSlowPivot() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setPivotMotorPercent(0.01);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.setPivotMotorPercent(0.02);
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            e.setPivotMotorPercent(0.025);
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            e.setPivotMotorPercent(-0.01);
        } else if (oi.isJoystickButtonPressed(0, 5)) {
            e.setPivotMotorPercent(-0.02);
        } else if (oi.isJoystickButtonPressed(0, 6)) {
            e.setPivotMotorPercent(-0.015);
        } else {
            e.stopPivotMotor();
        }
    }

    public void testAimPivot() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.movePivotToAngle(40);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.movePivotToAngle(70);
        } else {
            e.stopPivotMotor();
        }
    }

    public void testTuneMotor() {
        double joystickValue = -oi.getJoystickAxis(0, 1);
        int joystickPOV = oi.getPOV(0);
        RobotMotor motor = e.getSwerveDriveMotor(1);
        double maxVelocity = motor.getPercentVelocityLinearMapper().getMaxInputValue();
        double maxPercentPerSecond = 0.5;
        double maxVelocityPerSecond = maxVelocity / 2;

        if (isButtonPressed) {
            double timeValue = RobotMath.round2(testTimer.get());
            double lastPercent = motor.getLastAssignedPercentage();
            double lastVelocity = motor.getLastAssignedVelocity();
            double actualVelocity = RobotMath.round2(motor.getEncoderVelocity());
            if (lastPercent != 0.0) {
                rLog.print("Time: " + timeValue +
                        " Percent: " + lastPercent +
                        " Actual Velocity:" + actualVelocity);
            } else if (lastVelocity != 0.0) {
                double velocityError = RobotMath.round2((motor.getEncoderVelocity() -
                        motor.getLastAssignedVelocity()));
                rLog.print("Time: " + timeValue +
                        " Velocity: " + lastVelocity +
                        " Actual Velocity:" + actualVelocity +
                        " Err: " + velocityError);
            }
        }

        isButtonPressed = true;

        if (oi.isJoystickButtonPressed(0, 1)) {
            // Button 1 pressed - percentage output based on joystick
            motor.setPercent(joystickValue);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            // Button 2, assign percentage output based on POV
            switch (joystickPOV) {
                case 0:
                    // Button 2, POV=0 = percent output to 9%
                    motor.rampToPercent(0.03, maxPercentPerSecond);
                    break;
                case 45:
                    // Button 2, POV=45 = percent output to 12%
                    motor.rampToPercent(0.1, maxPercentPerSecond);
                    break;
                case 90:
                    // Button 2, POV=90
                    motor.rampToPercent(0.3, maxPercentPerSecond);
                    break;
                case 135:
                    // Button 2, POV=135
                    motor.rampToPercent(0.4, maxPercentPerSecond);
                    break;
                case 180:
                    // Button 2, POV=180
                    motor.rampToPercent(0.45, maxPercentPerSecond);
                    break;
                case 225:
                    // Button 2, POV=225 = percent output to 50%
                    motor.rampToPercent(0.8, maxPercentPerSecond);
                    break;
                case 270:
                    // Button 2, POV=270 = percent output to 75%
                    motor.rampToPercent(0.9, maxPercentPerSecond);
                    break;
                case 315:
                    // Button 2, POV=315 = percent output to 100%
                    motor.rampToPercent(0.94, maxPercentPerSecond);
                    break;
                default:
                    // No POV button pressed - turn off motors
                    isButtonPressed = false;
                    motor.rampToPercent(0.0, maxPercentPerSecond);
            }
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            // Button 3, assign velocity based on POV
            switch (joystickPOV) {
                case 0:
                    // Button 3, POV=0 = velocity to minimum speed
                    motor.rampToVelocity(0.04 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 45:
                    // Button 3, POV=45 = velocity to second lowest speed
                    motor.rampToVelocity(0.07 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 90:
                    // Button 3, POV=90 = velocity to third lowest speed
                    motor.rampToVelocity(0.12 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 135:
                    // Button 3, POV=135 = velocity to fourth lowest speed
                    motor.rampToVelocity(0.20 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 180:
                    // Button 3, POV=180 = velocity to fifth lowest speed
                    motor.rampToVelocity(0.45 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 225:
                    // Button 3, POV=225 = velocity to sixth lowest speed
                    motor.rampToVelocity(0.60 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 270:
                    // Button 3, POV=270 = velocity to 75%
                    motor.rampToVelocity(0.75 * maxVelocity, maxVelocityPerSecond);
                    break;
                case 315:
                    // Button 3, POV=315 = velocity to 100%
                    motor.rampToVelocity(1.0 * maxVelocity, maxVelocityPerSecond);
                    break;
                default:
                    // No POV button pressed - turn off motors
                    isButtonPressed = false;
                    motor.rampToVelocity(0.0, maxVelocityPerSecond);
            }
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            // Button 4 pressed - velocity control based on joystick
            motor.setVelocity(joystickValue * maxVelocity);
        } else {
            // No buttons pressed - turn off motors
            isButtonPressed = false;
            motor.stopMotor();
            motor.setIntegralAccumulator(0);
        }
    }

    public void TestLEDregions() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setLED(LEDregion.RobotStatus, LEDcolor.Green);
        } else {
            e.setLED(LEDregion.RobotStatus, LEDcolor.Black);
        }
        if (oi.isJoystickButtonPressed(0, 2)) {
            e.setLED(LEDregion.BellyPan, LEDcolor.Green);
        } else {
            e.setLED(LEDregion.BellyPan, LEDcolor.Black);
        }
        if (oi.isJoystickButtonPressed(0, 3)) {
            e.setLED(LEDregion.Alliance, LEDcolor.Green);
        } else {
            e.setLED(LEDregion.Alliance, LEDcolor.Black);
        }
        if (oi.isJoystickButtonPressed(0, 4)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Green);
        } else {
            e.setLED(LEDregion.RegionAll, LEDcolor.Black);
        }
    }

    public void TestLEDColors() {
        if (oi.isJoystickButtonPressed(0, 1)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.BadRed);
        } else if (oi.isJoystickButtonPressed(0, 2)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Blue);
        } else if (oi.isJoystickButtonPressed(0, 3)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.DimBlue);
        } else if (oi.isJoystickButtonPressed(0, 4)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.DimRed);
        } else if (oi.isJoystickButtonPressed(0, 5)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.DimWhite);
        } else if (oi.isJoystickButtonPressed(0, 6)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Green);
        } else if (oi.isJoystickButtonPressed(0, 7)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.GreenBlue);
        } else if (oi.isJoystickButtonPressed(0, 8)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Orange);
        } else if (oi.isJoystickButtonPressed(0, 9)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Purple);
        } else if (oi.isJoystickButtonPressed(0, 10)) {
            e.setLED(LEDregion.RegionAll, LEDcolor.Red);
        } else {
            e.setLED(LEDregion.RegionAll, LEDcolor.Black);
        }
    }

    public void ResetSwerveEncoder() {
        boolean isPressed = oi.isJoystickButtonPressed(0, 1);
        boolean zeroPressed = oi.isJoystickButtonPressed(0, 2);
        if (isPressed && !lastButtonPressed) {
            e.zeroAllAbsoluteEncoderValues(); // reset swerve encoders
            e.assignAllSwerveEncoderValues();
            rLog.print("Reset Swerve Encoders pressed");
        }
        if (zeroPressed && !lastZeroButtonPressed) {
            e.assignAllSwerveEncoderValues();
            rLog.print("Assign All Swerve Encoders pressed");
        }
        lastButtonPressed = isPressed;
        lastZeroButtonPressed = zeroPressed;
    }

    public void ResetPivotEncoder() {
        boolean isPressed = oi.isJoystickButtonPressed(0, 1);
        if (isPressed && !lastButtonPressed) {
            e.setPivotEncoderAngle(17.93);
            rLog.print("Set Pivot Endcoder pressed");
        }
        lastButtonPressed = isPressed;
    }
}