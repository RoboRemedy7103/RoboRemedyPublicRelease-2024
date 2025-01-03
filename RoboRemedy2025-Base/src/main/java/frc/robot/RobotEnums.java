// RobotEnums.java - Enums used by robot project
package frc.robot;

public class RobotEnums {
    public enum RobotName {
        JerryJr,
        Botchendo,
        LIVE2025,
    }

    public enum AutoStartPosition {
        Left,
        Right,
    }

    public enum TeleopProgram {
        TeleopSample,
        None,
    }

    public enum AutoProgram {
        JustAlign,
        DriveOut,
        SimulationProject,
        None,
    }

    public enum TestMode {
        QuickTest,
        SwerveTest,
        SwerveDrive,
        TestFacing,
        TestMotors,
        TestMotorsWJoystick,
        TuneMotor,
        TestLEDRegions,
        TestLEDColors,
        ResetSwerveEncoders,
        ResetPivotEncoder,
        TestCountdown,
        SlowPivot,
        AimPivot,
        TestNone
    }

    public enum LEDregion {
        RobotStatus,
        BellyPan,
        Alliance,
        AutoStep,
        RegionAll
    }

    public enum LEDcolor {
        BadRed,
        Red,
        DimRed,
        Green,
        GreenBlue,
        Blue,
        DimBlue,
        Yellow,
        Black,
        Purple,
        White,
        DimWhite,
        Orange,
        PastelRed,
        PastelYellow,
        PastelGreen,
        PastelBlue,
        PastelPurple
    }
}
