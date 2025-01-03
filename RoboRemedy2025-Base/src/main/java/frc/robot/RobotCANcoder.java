// RobotMotor.java - Class to be used for any type of motor
package frc.robot;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.measure.Angle;

public class RobotCANcoder {

    CANcoder canCoder;
    RoboLog rLog;
    MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
    public final double ERROR_ANGLE = -3600;

    RobotCANcoder(int id, boolean isInverted, RoboLog rLog) {
        this.rLog = rLog;
        canCoder = new CANcoder(id);
        canCoder.getConfigurator().refresh(magnetConfig);
        magnetConfig.SensorDirection = (isInverted ? SensorDirectionValue.Clockwise_Positive :
            SensorDirectionValue.CounterClockwise_Positive);
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
        canCoder.getConfigurator().apply(magnetConfig);
    }

    double getAbsoluteAngle() {
        StatusSignal<Angle> position = canCoder.getAbsolutePosition();
        StatusCode result = position.getStatus();
        if (result == StatusCode.OK)
            return position.getValueAsDouble() * 360.0;
        else
            return ERROR_ANGLE;
    }

    void setAbsoluteAngle(double angle) {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        canCoder.getConfigurator().refresh(magnetConfig);
        double magnetOffset = magnetConfig.MagnetOffset;
        double currentEncoderValue = canCoder.getAbsolutePosition().getValueAsDouble();
        magnetConfig.MagnetOffset = (magnetOffset + (angle / 360.0) - currentEncoderValue) % 1.0;
        rLog.print("Magnet offset changed from " + RobotMath.round2(magnetOffset) + " to " +
            RobotMath.round2(magnetConfig.MagnetOffset));
        canCoder.getConfigurator().apply(magnetConfig);
    }

    double getAngle() {
        StatusSignal<Angle> position = canCoder.getPosition();
        return position.getValueAsDouble() * 360.0;
    }

    void setAngle(double angle) {
        canCoder.setPosition(angle / 360.0);
    }
    public boolean isAttached() {
        StatusSignal<Angle> position = canCoder.getPosition();
        return position.getStatus().isOK();
    }
}