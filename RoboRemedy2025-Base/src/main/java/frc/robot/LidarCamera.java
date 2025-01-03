// LidarCamera.java - Controls a lidar camera on a Raspberry Pi
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.networktables.*;

public class LidarCamera {
    long lastCounter = 0;

    Timer changeTime = new Timer();

    final double TIME_OUT = 0.5;

    NetworkTable networkTable;
    DoublePublisher regionXStartPub;
    DoublePublisher regionXEndPub;
    DoublePublisher regionYStartPub;
    DoublePublisher regionYEndPub;

    DoubleSubscriber zSubscriber;
    IntegerSubscriber counterSubscriber;

    public LidarCamera(String networkTableName) {
        networkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
        regionXStartPub = networkTable.getDoubleTopic("regionXStart").publish();
        regionXEndPub = networkTable.getDoubleTopic("regionXEnd").publish();
        regionYStartPub = networkTable.getDoubleTopic("regionYStart").publish();
        regionYEndPub = networkTable.getDoubleTopic("regionYEnd").publish();

        zSubscriber = networkTable.getDoubleTopic("z").subscribe(-1.0);
        counterSubscriber = networkTable.getIntegerTopic("counter").subscribe(-1);

        changeTime.restart();
    }

    /**
     * Sets the lidar's range for checking data points. This one will set a 2d grid for the range of the lidar.
     * 
     * @param regionXStart The beginning of the x (horizontal) range that the lidar will check.
     * @param regionXEnd The end of the x (horizontal) range that the lidar will check.
     * @param regionYStart The bottom of the y (vertical) range that the lidar will check.
     * @param regionYEnd The top of the y (vertical) range that the lidar will check.
     * 
     * <p> Note that if the end number's smaller than the start number, they will be reversed.
     */
    public void setRegion(double regionXStart, double regionXEnd, double regionYStart, double regionYEnd) {
        if (regionXEnd < regionXStart) {
            regionXStartPub.set(regionXEnd);
            regionXEndPub.set(regionXStart);
        } else {
            regionXStartPub.set(regionXStart);
            regionXEndPub.set(regionXEnd);
        }
        if (regionYEnd < regionYStart) {
            regionYStartPub.set(regionYEnd);
            regionYEndPub.set(regionYStart);
        } else {
            regionYStartPub.set(regionYStart);
            regionYEndPub.set(regionYEnd);
        }
    }

    /**
     * Sets the lidar's range for checking data points. This one will set a line across the x axis for a specified 
     * range at the y value.
     * 
     * @param regionXStart The beginning of the x (horizontal) range that the lidar will check.
     * @param regionXEnd The end of the x (horizontal) range that the lidar will check.
     * @param regionY The y value that will be checked.
     * 
     * <p> Note that if the end number's smaller than the start number, they will be reversed.
     */
    public void setRegion(double regionXStart, double regionXEnd, double regionY) {
        setRegion(regionXStart, regionXEnd, regionY, regionY);
    }

    /**
     * Sets the lidar's range for checking data points. This one will set a single point.
     * 
     * @param regionX The x component of the point.
     * @param regionY The y component of the point.
     */
    public void setRegion(double regionX, double regionY) {
        setRegion(regionX, regionX, regionY, regionY);
    }

    public double getRegionZ() {
        return zSubscriber.get();
    }

    public boolean isConnected() {
        long counterValue = counterSubscriber.get();
        if (counterValue == -1)
            return false;

        double currentTime = changeTime.get();
        if (counterValue != lastCounter) {
            lastCounter = counterValue;
            changeTime.restart();
            return true;
        } else {
            if (currentTime <= TIME_OUT) {
                return true;
            } else {
                return false;
            }
        }
    }
}
