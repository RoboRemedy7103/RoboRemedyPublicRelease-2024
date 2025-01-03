// MeasureConvert.java - Converts between different units
package frc.robot;

public class MeasureConvert {
    public static enum Unit {
        Meter,
        Centimeter,
        Millimeter,
        Yard,
        Foot,
        Inch,
    }
    
    //All values used for conversion are equal to 1 meter
    private static final double METER = 1;
    private static final double CENTIMETER = .01;
    private static final double MILLIMETER = .001;
    private static final double YARD = 1.09361;
    private static final double FOOT = 3.28084;
    private static final double INCH = 39.3701;

    public static double convertMeasurement(double value, Unit valueUnit, Unit desiredUnit) {
        double valueConversion;
        double desiredConversion;
        switch (valueUnit) {
            case Meter:
                valueConversion = METER;
                break;
            case Centimeter:
                valueConversion = CENTIMETER;
                break;
            case Millimeter:
                valueConversion = MILLIMETER;
                break;
            case Yard:
                valueConversion = YARD;
                break;
            case Foot:
                valueConversion = FOOT;
                break;
            case Inch:
                valueConversion = INCH;
                break;
            default:
                valueConversion = 1;
                break;
        }
        switch (desiredUnit) {
            case Meter:
                desiredConversion = METER;
                break;
            case Centimeter:
                desiredConversion = CENTIMETER;
                break;
            case Millimeter:
                desiredConversion = MILLIMETER;
                break;
            case Yard:
                desiredConversion = YARD;
                break;
            case Foot:
                desiredConversion = FOOT;
                break;
            case Inch:
                desiredConversion = INCH;
                break;
            default:
                desiredConversion = 1;
                break;
        }
        return value * (desiredConversion / valueConversion);
    }
}
