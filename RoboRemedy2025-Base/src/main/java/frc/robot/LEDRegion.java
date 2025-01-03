// LEDRegion.java - Controls a single section of an LED String
package frc.robot;

public class LEDRegion {

    private RoboLog rLog;
    private LEDString ledString;
    private int begin;
    private int end;

    LEDRegion(RoboLog rLog, LEDString ledString, int begin, int end) {
        this.rLog = rLog;
        this.ledString = ledString;
        this.begin = begin;
        this.end = end;

        if (this.rLog == null) {
            System.out.println("Warning: LEDRegion.rLog is null");
        }
        if (this.ledString == null) {
            rLog.print("Warning: LEDRegion.ledString is null");
        }
        if (this.begin < 0) {
            rLog.print("Warning: LEDRegion.begin < 0");
        }
        if (this.end < 0) {
            rLog.print("Warning: LEDRegion.end < 0");
        }
    }
}
