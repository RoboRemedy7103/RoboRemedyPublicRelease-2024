// RobotGyro.java - Adds an offset to the NavX gyro
package frc.robot;

import com.studica.frc.*;

public class RobotGyro extends AHRS {

    private double gyroOffset = 0;
    private boolean isSimulation = false;
    private float simulationAngle = 0;

    RobotGyro() {
        super(AHRS.NavXComType.kMXP_SPI);
    }

    @Override
    public float getYaw() {
        if (isSimulation)
            return simulationAngle;
        else
            return super.getYaw() - (float)gyroOffset;
    }

    public void setYaw(double yawAngle) {
        gyroOffset = super.getYaw() - yawAngle;
    }

    public void setSimulationYaw(double yawAngle) {
        isSimulation = true;
        simulationAngle = (float)yawAngle;
    }
}