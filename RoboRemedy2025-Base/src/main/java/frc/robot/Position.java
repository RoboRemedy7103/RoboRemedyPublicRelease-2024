package frc.robot;

/**
 * A position class whose values are not static so variables can be changed without creating a new object. 
 * Values are in inches and degrees.
 */
public class Position {
    private double x = Double.MAX_VALUE;
    private double y = Double.MAX_VALUE;
    private double z = Double.MAX_VALUE;
    private double yaw = Double.MAX_VALUE;
    private double pitch = Double.MAX_VALUE;
    private double roll = Double.MAX_VALUE;
    private final String POSITION_NAME;

    /**
     * Creates a blank Position. It's highly encouraged to add a string as an argument unless it is, in fact, expendable.
     */
    Position() {
        POSITION_NAME = "EXPENDABLE";
    }

    /**
     * Creates a blank Position with a name.
     * @param positionName
     */
    Position(String positionName) {
        POSITION_NAME = positionName;
    }

    /**
     * Creates a new Position with x and y values. Other values will be left at Double's maximum value.
     * @param x     The x value of the position.
     * @param y     The y value of the position.
     * @param yaw   The z value of the position.
     */
    Position(String positionName, double x, double y) {
        this.x = x;
        this.y = y;
        this.POSITION_NAME = positionName;
    }

    /**
     * Creates a new Position with x, y, and yaw values. Other values will be left at Double's maximum value.
     * @param x     The x value of the position.
     * @param y     The y value of the position.
     * @param yaw   The yaw of the position.
     */
    Position(String positionName, double x, double y, double yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
        this.POSITION_NAME = positionName;
    }

    /**
     * Creates a new Position with x, y, and z values, as well as yaw, pitch, and roll values. Other values will be left at Double's maximum value.
     * @param x     The x value of the position.
     * @param y     The y value of the position.
     * @param z     The z value of the position.
     * @param yaw   The yaw of the position.
     * @param pitch The pitch of the position.
     * @param roll  The roll of the position.
     */
    Position(String positionName, double x, double y, double z, double yaw, double pitch, double roll) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.POSITION_NAME = positionName;
    }

    public double getX() {
        if (x == Double.MAX_VALUE)
            System.out.println("WARNING: X Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return x;
    }

    public double getY() {
        if (y == Double.MAX_VALUE)
            System.out.println("WARNING: Y Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return y;
    }

    public double getZ() {
        if (z == Double.MAX_VALUE)
            System.out.println("WARNING: X Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return z;
    }

    public double getYaw() {
        if (yaw == Double.MAX_VALUE)
            System.out.println("WARNING: Yaw Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return yaw;
    }

    public double getPitch() {
        if (pitch == Double.MAX_VALUE)
            System.out.println("WARNING: Pitch Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return pitch;
    }

    public double getRoll() {
        if (roll == Double.MAX_VALUE)
            System.out.println("WARNING: X Value is not assigned properly in " + POSITION_NAME + ". Check that it's being assigned properly.");
        return roll;
    }

    public String getName() {
        return POSITION_NAME;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public void setYaw(double yaw) {
        double yawValue = yaw;
        while (yawValue > 360)
            yawValue -= 360;
        while (yawValue < 0)
            yawValue += 360;
        this.yaw = yawValue;
    }

    public void setPitch(double pitch) {
        double pitchValue = pitch;
        while (pitchValue > 360)
            pitchValue -= 360;
        while (pitchValue < 0)
            pitchValue += 360;
        this.pitch = pitchValue;
    }

    public void setRoll(double roll) {
        double rollValue = roll;
        while (rollValue > 360)
            rollValue -= 360;
        while (rollValue < 0)
            rollValue += 360;
        this.roll = rollValue;
    }

    public void setPosition(double x, double y) {
        setX(x);
        setY(y);
    }

    public void setPosition(double x, double y, double yaw) {
        setX(x);
        setY(y);
        setYaw(yaw);
    }

    public void setPosition(double x, double y, double z, double yaw, double pitch, double roll) {
        setX(x);
        setY(y);
        setZ(z);
        setYaw(yaw);
        setPitch(pitch);
        setRoll(roll);
    }

    public void addToPosition(double x, double y) {
        this.x += x;
        this.y += y;
    }
}
