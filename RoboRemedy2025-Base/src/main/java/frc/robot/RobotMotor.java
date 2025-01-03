// RobotMotor.java - Class to be used for any type of motor
package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.SparkClosedLoopController.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

public class RobotMotor {
    public enum RobotMotorType {
        SparkMax, TalonFX, TalonSRX, Victor, SparkFlex,
    }

    public enum RobotEncoderType {
        None, Internal, Cancoder, MagEncoder, ThroughBore,
    }

    TalonFXConfiguration fxConfigs = new TalonFXConfiguration();
    private boolean fxNeedsConfigsSet = false;

    private RoboLog rLog;
    private int motorID;
    private RobotMotorType motorType;
    private SparkBase sparkMotor;
    private SparkBaseConfig sparkConfigs;
    private RelativeEncoder sparkEncoder;
    private SparkClosedLoopController sparkPID;
    private TalonFX talonFXMotor;
    private TalonSRX talonSRXMotor;
    private VictorSPX victorMotor;
    private BaseMotorController baseCtreMotor;
    private VoltageOut voltageOut = new VoltageOut(0);
    private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private boolean isUsingVoltageCompensation;
    private PositionVoltage positionOut = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0).withSlot(0);
    private Timer timer = new Timer();
    private Timer sparkTimer = new Timer();

    private LinearMapper mapper = null;
    private double unitsPerEncoderTick = 1.0;
    private double encoderTicksPerUnit = 1.0;
    private double lastAssignedPercentage = 0;
    private double lastAssignedPosition = 0;
    private double lastAssignedVelocity = 0;
    private double lastAssignedFeedForward = 0;
    private double lastAssignedTime = 0;
    private boolean isCoast;

    RobotMotor(RobotMotorType motorType, int motorID, boolean isInverted, boolean isCoast, RoboLog rLog,
            double unitsPerRevolution, double maxCurrent, double statorCurrent, RobotEncoderType encoderType, 
            int absoluteEncoderID, boolean isUsingVoltageCompensation) {
        this.rLog = rLog;
        this.motorID = motorID;
        this.motorType = motorType;
        this.isCoast = isCoast;
        this.isUsingVoltageCompensation = isUsingVoltageCompensation;
        timer.restart();
        sparkTimer.restart();
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            if (motorType == RobotMotorType.SparkMax) {
                sparkMotor = new SparkMax(motorID, MotorType.kBrushless);
                sparkConfigs = new SparkMaxConfig();
            } else {
                sparkMotor = new SparkFlex(motorID, MotorType.kBrushless);
                sparkConfigs = new SparkFlexConfig();
            }
            sparkEncoder = sparkMotor.getEncoder();
            sparkPID = sparkMotor.getClosedLoopController();
            sparkConfigs.inverted(isInverted);
            sparkConfigs.idleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
            sparkConfigs.smartCurrentLimit((int) maxCurrent);
            sparkConfigs.encoder.velocityConversionFactor(unitsPerRevolution / 60.0);
            sparkConfigs.encoder.positionConversionFactor(unitsPerRevolution);
            sparkConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            sparkConfigs.closedLoop.pid(0, 0, 0);
            REVLibError result = sparkMotor.configure(sparkConfigs, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
            if (result != REVLibError.kOk) {
                rLog.print("Error setting configuration. SparkID=" + motorID + ", Error=" + result.toString());
                fxNeedsConfigsSet = true;
            } else {
                rLog.print("Setting configuration Success. SparkID=" + motorID);
                fxNeedsConfigsSet = false;
            }
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor = new TalonFX(motorID);
            fxConfigs.MotorOutput.Inverted = (isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
            fxConfigs.MotorOutput.NeutralMode = (isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            fxConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.00;
            fxConfigs.CurrentLimits.SupplyCurrentLimitEnable = maxCurrent == 0 ? false : true;
            fxConfigs.CurrentLimits.SupplyCurrentLimit = maxCurrent;
            fxConfigs.CurrentLimits.SupplyCurrentLowerLimit = maxCurrent;
            fxConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;
            fxConfigs.CurrentLimits.StatorCurrentLimitEnable = statorCurrent == 0 ? false : true;
            fxConfigs.CurrentLimits.StatorCurrentLimit = statorCurrent;
            if (encoderType == RobotEncoderType.None) {

            } else if (encoderType == RobotEncoderType.Internal) {
                fxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                fxConfigs.Feedback.SensorToMechanismRatio = 1.0 / unitsPerRevolution;
            } else if (encoderType == RobotEncoderType.Cancoder) {
                fxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                fxConfigs.Feedback.FeedbackRemoteSensorID = absoluteEncoderID;
            } else {
                rLog.print("Unsupported Encoder Type: " + encoderType);
            }
            StatusCode result = talonFXMotor.getConfigurator().apply(fxConfigs);
            if (!result.isOK()) {
                rLog.print("Error setting configuration. TalonID=" + motorID + ", Code=" + result.getDescription());
                fxNeedsConfigsSet = true;
            } else {
                rLog.print("Setting configuration Success. TalonID=" + motorID);
                fxNeedsConfigsSet = false;
            }
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            fxNeedsConfigsSet = false;
            if (motorType == RobotMotorType.TalonSRX) {
                talonSRXMotor = new TalonSRX(motorID);
                baseCtreMotor = talonSRXMotor;
            } else if (motorType == RobotMotorType.Victor) {
                victorMotor = new VictorSPX(motorID);
                baseCtreMotor = victorMotor;
            }
            baseCtreMotor.configFactoryDefault();
            baseCtreMotor.setInverted(isInverted);
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
            baseCtreMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
            baseCtreMotor.configVelocityMeasurementWindow(1);
            baseCtreMotor.configNeutralDeadband(0.01);
            baseCtreMotor.configVoltageCompSaturation(12.0);
            baseCtreMotor.enableVoltageCompensation(true);
            setInputCurrentLimit(maxCurrent);
            if (encoderType == RobotEncoderType.None) {

            } else if (encoderType == RobotEncoderType.Internal) {

            } else if (encoderType == RobotEncoderType.Cancoder) {
                baseCtreMotor.configRemoteFeedbackFilter(absoluteEncoderID, RemoteSensorSource.CANCoder, 0);
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.MagEncoder) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                baseCtreMotor.setSensorPhase(true);
                unitsPerEncoderTick = unitsPerRevolution / 4096;
            } else if (encoderType == RobotEncoderType.ThroughBore) {
                baseCtreMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                unitsPerEncoderTick = unitsPerRevolution / 8192;
            }
            encoderTicksPerUnit = 1.0 / unitsPerEncoderTick;
        } else {
            this.rLog.print("RobotMotor created with unknown RobotMotorType");
        }
    }

    public RobotMotorType getMotorType() {
        return motorType;
    }

    public int getMotorID() {
        return motorID;
    }

    public void setInputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            fxConfigs.CurrentLimits.SupplyCurrentLimit = amps;
            fxConfigs.CurrentLimits.SupplyCurrentLowerLimit = amps;
            fxConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.0;
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX) {
            SupplyCurrentLimitConfiguration talonCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, amps,
                    amps, 0.2);
            talonSRXMotor.configSupplyCurrentLimit(talonCurrentLimitConfig);
        }
    }

    public void setOutputCurrentLimit(double amps) {
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            sparkConfigs.smartCurrentLimit((int) amps);
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            fxConfigs.CurrentLimits.StatorCurrentLimit = amps;
            talonFXMotor.getConfigurator().apply(fxConfigs);
        }
    }

    public double getEncoderPosition() {
        if (motorType == RobotMotorType.SparkFlex
                || motorType == RobotMotorType.SparkMax) {
            return sparkEncoder.getPosition();
        } else if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.getPosition().getValueAsDouble();
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorPosition() * unitsPerEncoderTick;
        } else {
            return 0;
        }
    }

    public boolean isEncoderAttached() {
        if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.getPosition().getStatus().isOK();
        } else {
            return false;
        }
    }

    public boolean isFXConfigNeeded() {
        return fxNeedsConfigsSet;
    }

    public boolean configReapply() {
        if (motorType == RobotMotorType.SparkMax) {
            return true;
        } else if (motorType == RobotMotorType.SparkFlex) {
            return true;
        } else if (motorType == RobotMotorType.TalonFX) {
            StatusCode result = talonFXMotor.getConfigurator().apply(fxConfigs);
            if (!result.isOK()) {
                rLog.print("Error setting configuration. TalonID=" + motorID + ", Code=" + result.getDescription());
                fxNeedsConfigsSet = true;
                return false;
            } else {
                rLog.print("Setting configuration Success. TalonID=" + motorID);
                fxNeedsConfigsSet = false;
                return true;
            }
        } else if (motorType == RobotMotorType.TalonSRX || motorType == RobotMotorType.Victor) {
            return true;
        } else {
            return false;
        }
    }

    public double getStatorCurrent() {
        return motorType == RobotMotorType.TalonFX ?
            talonFXMotor.getStatorCurrent().getValue().in(Units.Amps) : 0;
    }

    public double getEncoderVelocity() {
        if (motorType == RobotMotorType.SparkMax
                || motorType ==RobotMotorType.SparkFlex) {
            return sparkEncoder.getVelocity();
        } else if (motorType == RobotMotorType.TalonFX) {
            return talonFXMotor.getVelocity().getValueAsDouble();
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            return baseCtreMotor.getSelectedSensorVelocity() * unitsPerEncoderTick * 10;
        } else {
            return 0;
        }
    }

    /** Assign the value to be used as the current encoder position
    This does not move the motor. Use setPosition to move the motor. */
    public boolean setEncoderValue(double position) {
        if (motorType == RobotMotorType.SparkMax 
                || motorType == RobotMotorType.SparkFlex) {
            REVLibError result = sparkEncoder.setPosition(position);
            if (!(result == REVLibError.kOk)) {
                rLog.print("Error in setEncoderValue. SparkID=" + motorID + ", Code=" + REVLibError.fromInt(result.value));
                return false;
            }
            return true;
        } else if (motorType == RobotMotorType.TalonFX) {
            StatusCode result = talonFXMotor.setPosition(position);
            if (!result.isOK()) {
                rLog.print("Error in setEncoderValue. TalonID=" + motorID + ", Code=" + result.getDescription());
                return false;
            }
            return true;
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            ErrorCode result = baseCtreMotor.setSelectedSensorPosition((int) (position * encoderTicksPerUnit));
            if (!(result == ErrorCode.OK)) {
                rLog.print("Error in setEncoderValue. SRX ID=" + motorID + ", Code=" + ErrorCode.valueOf(result.value));
                return false;
            }
            return true;
        }
        rLog.print("Motor type is invalid-encoder cannot be set");
        return false;
    }

    public void setPercentVelocityLinearMapper(LinearMapper mapper) {
        this.mapper = mapper;
    }

    public LinearMapper getPercentVelocityLinearMapper() {
        if (mapper == null)
            return new LinearMapper();
        else
            return mapper;
    }

    // Assign the PID values to be used for setPosition or setVelicty calls
    public void setPID(double p, double i, double d, double iZone, boolean isVelocity) {
        if (motorType == RobotMotorType.SparkMax
            || motorType == RobotMotorType.SparkFlex) {
            sparkConfigs.closedLoop.pid(p, i, d);
            sparkConfigs.closedLoop.iZone(iZone);
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.Slot0.kP = p * 2;
            fxConfigs.Slot0.kI = i * 2;
            fxConfigs.Slot0.kD = d * 2;
            // TalonFX does not need izone parameter
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            double scalingFactor = (isVelocity ? encoderTicksPerUnit * 0.1 : unitsPerEncoderTick);
            baseCtreMotor.config_kF(0, 0);
            baseCtreMotor.config_kP(0, p * scalingFactor);
            baseCtreMotor.config_kI(0, i * scalingFactor);
            baseCtreMotor.config_kD(0, d * scalingFactor);
            baseCtreMotor.config_IntegralZone(0, (int) (iZone * scalingFactor));
        }
    }

    public void setDriveOpenLoopRamp(double rate) {
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            sparkConfigs.openLoopRampRate(rate);
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rate;
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configOpenloopRamp(rate);
        }
    }

    public void setPercent(double percent) {
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            sparkMotor.set(percent);
        } else if (motorType == RobotMotorType.TalonFX) {
            if (isUsingVoltageCompensation) {
                talonFXMotor.setControl(voltageOut.withOutput(12.0 * percent));
            } else {
                talonFXMotor.setControl(dutyCycleOut.withOutput(percent));
            }
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.PercentOutput, percent);
        }
        lastAssignedPercentage = percent;
        lastAssignedPosition = 0;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    /***
     * Attempt to move the motor to a specific position using a PID loop
     * Use setEncoderValue to change the current encoder value without moving the motor
     * 
     * @param desiredPosition Target position
     */
    public void setPosition(double desiredPosition) {
        if (motorType == RobotMotorType.SparkMax
                || motorType == RobotMotorType.SparkFlex) {
            sparkPID.setReference(desiredPosition, SparkMax.ControlType.kPosition);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(positionOut.withPosition(desiredPosition));
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Position, (desiredPosition * encoderTicksPerUnit));
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = desiredPosition;
        lastAssignedVelocity = 0;
        lastAssignedFeedForward = 0;
        lastAssignedTime = timer.get();
    }

    /***
     * Attempt to move the motor at a specific velocity using a PID loop
     * 
     * @param velocity Target velocity
     */
    public void setVelocity(double velocity) {
        double feedForward = getPercentageOutputFromVelocity(velocity);
        if (motorType == RobotMotorType.SparkMax
            || motorType == RobotMotorType.SparkFlex) {
            // rLog.print("setVelocity:" + velocity + " AFF:" + feedForward);
            sparkPID.setReference(velocity, SparkMax.ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
        } else if (motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(velocityOut.withVelocity(velocity).withFeedForward(feedForward * 12.0));
        } else if(motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.set(ControlMode.Velocity, (velocity * encoderTicksPerUnit * 0.1),
                    DemandType.ArbitraryFeedForward, feedForward);
        }
        lastAssignedPercentage = 0;
        lastAssignedPosition = 0;
        lastAssignedVelocity = velocity;
        lastAssignedFeedForward = feedForward;
        lastAssignedTime = timer.get();
    }

    // Calculate the best guess of the voltage perctage output (from -1.0 to 1.0)
    // that
    // will be needed for a given velocity. This value can be used as a feed forward
    // value for a PID loop.
    private double getPercentageOutputFromVelocity(double velocity_In) {
        double percent_Out = 0;
        if (velocity_In >= 0) {
            if (velocity_In < 0.01) {
                percent_Out = 0;
            } else {
                if (mapper != null) {
                    percent_Out = mapper.calculate(velocity_In);
                } else
                    percent_Out = 0;
            }
        } else {
            percent_Out = (-1 * getPercentageOutputFromVelocity(-1 * velocity_In));
        }
        return percent_Out;
    }

    public void setMaxOutput(double percentOutput) {
        if (motorType == RobotMotorType.SparkMax
        || motorType == RobotMotorType.SparkFlex) {
            sparkConfigs.closedLoop.outputRange(-percentOutput, percentOutput);
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.Voltage.PeakForwardVoltage = 12.0 * percentOutput;
            fxConfigs.Voltage.PeakReverseVoltage = -12.0 * percentOutput;
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configClosedLoopPeakOutput(0, percentOutput);
        }
    }

    /** The Spark Max requires this method to be called after changing parameters*/
    public void burnFlash() {
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX || motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
        }
    }

    public void stopMotor() {
        setPercent(0);
    }

    public double getOutputPercent() {
        double outputPercent = 0;
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            outputPercent = sparkMotor.getAppliedOutput();
        } else if (motorType == RobotMotorType.TalonFX) {
            outputPercent = talonFXMotor.getMotorVoltage().getValue().in(Units.Volts) / 12.0;
        } else if(motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            outputPercent = baseCtreMotor.getMotorOutputPercent();
        }
        return outputPercent;
    }

    public double getLastAssignedPercentage() {
        return lastAssignedPercentage;
    }

    public double getLastAssignedVelocity() {
        return lastAssignedVelocity;
    }

    public double getLastAssignedPosition() {
        return lastAssignedPosition;
    }

    public double getLastAssignedFeedForward() {
        return lastAssignedFeedForward;
    }

    public void rampToPercent(double percent, double maxPercentPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxPercentPerSecond * timeBetween;
        percent = Math.max(Math.min(percent, lastAssignedPercentage + maxChange), lastAssignedPercentage - maxChange);
        setPercent(percent);
    }

    public void rampToVelocity(double velocity, double maxVelocityPerSecond) {
        double timeBetween = timer.get() - lastAssignedTime;
        double maxChange = maxVelocityPerSecond * timeBetween;
        velocity = Math.max(Math.min(velocity, lastAssignedVelocity + maxChange), lastAssignedVelocity - maxChange);
        setVelocity(velocity);
    }

    public void setNeutralModeDeadband(double percentDeadband) {
        if (motorType == RobotMotorType.SparkMax || motorType == RobotMotorType.SparkFlex) {
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.MotorOutput.DutyCycleNeutralDeadband = percentDeadband;
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.configNeutralDeadband(percentDeadband);
        }
    }

    public void setNeutralMode(boolean isCoast) {
        if (this.isCoast == isCoast)
            return;
        this.isCoast = isCoast;
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            sparkConfigs.idleMode(isCoast ? IdleMode.kCoast : IdleMode.kBrake);
            sparkMotor.configure(sparkConfigs, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        } else if (motorType == RobotMotorType.TalonFX) {
            fxConfigs.MotorOutput.NeutralMode = (isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
            talonFXMotor.getConfigurator().apply(fxConfigs);
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
        }
    }

    public void followMotor(RobotMotor motorToFollow) {
        if (motorToFollow.motorType == RobotMotorType.TalonFX && motorType == RobotMotorType.TalonFX) {
            talonFXMotor.setControl(new StrictFollower(motorToFollow.getMotorID()));
        } else if ((motorToFollow.motorType == RobotMotorType.TalonSRX
                    || motorToFollow.motorType == RobotMotorType.Victor)
                && (motorType == RobotMotorType.TalonSRX
                    || motorType == RobotMotorType.Victor)) {
            baseCtreMotor.follow(motorToFollow.baseCtreMotor);
        } else {
            rLog.print("Can not follow Motor type" + motorType + " " + motorToFollow.motorType);
        }
    }

    public void setIntegralAccumulator(double value) {
        if (motorType == RobotMotorType.SparkMax
                || motorType == RobotMotorType.SparkFlex) {
            sparkPID.setIAccum(value);
        } else if (motorType == RobotMotorType.TalonFX) {
            // TalonFX does not need to use integral accumulator
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            baseCtreMotor.setIntegralAccumulator(value);
        }
    }

    public boolean isAttached() {
        if (motorType == RobotMotorType.SparkMax ||
            motorType == RobotMotorType.SparkFlex) {
            return sparkMotor.hasActiveFault();
        } else if (motorType == RobotMotorType.TalonFX) {
            StatusSignal<Angle> position = talonFXMotor.getPosition();
            return position.getStatus().isOK();
        } else if (motorType == RobotMotorType.TalonSRX
                || motorType == RobotMotorType.Victor) {
            if (baseCtreMotor.getLastError() != ErrorCode.CAN_MSG_NOT_FOUND) {
                return true;
            }
        }
        return false;
    }
}