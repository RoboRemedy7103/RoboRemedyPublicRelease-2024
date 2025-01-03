// RobotLog.java - Used to print information to the log file
package frc.robot;

import edu.wpi.first.wpilibj.*;
import java.io.*;

public class RoboLog {
    private Timer timer = new Timer();
    private String shortDescription = "INIT";

    public RoboLog() {
        timer.restart();
    }

    public void setRobotMode(String robotMode, String longDescription) {
        print("New Mode: ", longDescription);
        shortDescription = robotMode;
        timer.reset();
    }

    public void print(String s) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription);
        System.out.print(": ");
        System.out.println(s);
    }

    public void print(double s) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription);
        System.out.print(": ");
        System.out.println(s);
    }

    public void print(String s1, String s2) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription);
        System.out.print(": ");
        System.out.print(s1);
        System.out.println(s2);
    }

    public void printf(String format, Object... arguments) {
        System.out.printf("%3.3f ", timer.get());
        System.out.print(shortDescription);
        System.out.print(": ");
        System.out.printf(format, arguments);
        System.out.println();
    }

    public void printException(Exception e) {
        try {
            e.printStackTrace(new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt"), true));
        } catch (Exception o) {

        }
    }
}