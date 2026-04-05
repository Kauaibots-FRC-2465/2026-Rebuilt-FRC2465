package frc.robot.subsystems;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;

/**
 * Aggregate subsystem for the coordinated shooter flywheels.
 *
 * <p>This subsystem owns the paired main flywheels, kicker, and backspin flywheel
 * so they can be scheduled as a single unit.
 */
public class ShooterSubsystem extends SubsystemBase {
    /**
     * Configuration for one internally owned flywheel.
     */
    public static final class FlywheelConfig {
        final int canId;
        final String motorName;
        final double flywheelDiameterInches;
        final double gearRatio;
        final double kS;
        final double kV;
        final double kP;
        final double peakCurrent;
        final boolean motorReversed;
        final double velocityFilterTimeConstantSeconds;

        public FlywheelConfig(
                int canId,
                String motorName,
                double flywheelDiameterInches,
                double gearRatio,
                double kS,
                double kV,
                double kP,
                double peakCurrent) {
            this(
                    canId,
                    motorName,
                    flywheelDiameterInches,
                    gearRatio,
                    kS,
                    kV,
                    kP,
                    peakCurrent,
                    false,
                    Double.NaN);
        }

        public FlywheelConfig(
                int canId,
                String motorName,
                double flywheelDiameterInches,
                double gearRatio,
                double kS,
                double kV,
                double kP,
                double peakCurrent,
                boolean motorReversed) {
            this(
                    canId,
                    motorName,
                    flywheelDiameterInches,
                    gearRatio,
                    kS,
                    kV,
                    kP,
                    peakCurrent,
                    motorReversed,
                    Double.NaN);
        }

        public FlywheelConfig(
                int canId,
                String motorName,
                double flywheelDiameterInches,
                double gearRatio,
                double kS,
                double kV,
                double kP,
                double peakCurrent,
                boolean motorReversed,
                double velocityFilterTimeConstantSeconds) {
            this.canId = canId;
            this.motorName = Objects.requireNonNull(motorName, "motorName must not be null");
            this.flywheelDiameterInches = flywheelDiameterInches;
            this.gearRatio = gearRatio;
            this.kS = kS;
            this.kV = kV;
            this.kP = kP;
            this.peakCurrent = peakCurrent;
            this.motorReversed = motorReversed;
            this.velocityFilterTimeConstantSeconds = velocityFilterTimeConstantSeconds;
        }
    }

    private final KrakenFlywheelSubsystem mainFlywheel;
    private final KrakenFlywheelSubsystem mirroredMainFlywheel;
    private final KrakenFlywheelSubsystem kicker;
    private final KrakenFlywheelSubsystem backspinFlywheel;

    public ShooterSubsystem(
            String canBusName,
            FlywheelConfig mainFlywheelConfig,
            FlywheelConfig mirroredMainFlywheelConfig,
            FlywheelConfig kickerConfig,
            FlywheelConfig backspinConfig) {
        Objects.requireNonNull(canBusName, "canBusName must not be null");
        Objects.requireNonNull(mainFlywheelConfig, "mainFlywheelConfig must not be null");
        Objects.requireNonNull(mirroredMainFlywheelConfig, "mirroredMainFlywheelConfig must not be null");
        Objects.requireNonNull(kickerConfig, "kickerConfig must not be null");
        Objects.requireNonNull(backspinConfig, "backspinConfig must not be null");

        mainFlywheel = createFlywheel(canBusName, mainFlywheelConfig);
        mirroredMainFlywheel = createFlywheel(canBusName, mirroredMainFlywheelConfig);
        kicker = createFlywheel(canBusName, kickerConfig);
        backspinFlywheel = createFlywheel(canBusName, backspinConfig);
    }

    /**
     * Sets each shooter flywheel to its requested surface speed.
     */
    public void setIPS(double mainFlywheelIps, double kickerIps, double backspinIps) {
        mainFlywheel.setIPS(mainFlywheelIps);
        mirroredMainFlywheel.setIPS(mainFlywheelIps);
        kicker.setIPS(kickerIps*.7);
        backspinFlywheel.setIPS(backspinIps);
    }

    /**
     * Sets a symmetric shooter speed where kicker runs opposite the main and
     * backspin flywheels.
     */
    public void setCoupledIPS(double ips) {
        setIPS(ips, -ips, ips);
    }

    /**
     * Returns a command that continuously drives each shooter flywheel to the
     * supplied surface speed.
     */
    public Command cmdSetIPS(
            DoubleSupplier mainFlywheelIpsSupplier,
            DoubleSupplier kickerIpsSupplier,
            DoubleSupplier backspinIpsSupplier) {
        Objects.requireNonNull(mainFlywheelIpsSupplier, "mainFlywheelIpsSupplier must not be null");
        Objects.requireNonNull(kickerIpsSupplier, "kickerIpsSupplier must not be null");
        Objects.requireNonNull(backspinIpsSupplier, "backspinIpsSupplier must not be null");

        return new OverrideCommand(this) {
            @Override
            public void execute() {
                setIPS(
                        mainFlywheelIpsSupplier.getAsDouble(),
                        kickerIpsSupplier.getAsDouble(),
                        backspinIpsSupplier.getAsDouble());
            }
        };
    }

    /**
     * Returns a command that continuously drives the shooter in the common
     * coupled mode: main and backspin positive, kicker negative.
     */
    public Command cmdSetCoupledIPS(DoubleSupplier ipsSupplier) {
        Objects.requireNonNull(ipsSupplier, "ipsSupplier must not be null");
        return new OverrideCommand(this) {
            @Override
            public void execute() {
                setCoupledIPS(ipsSupplier.getAsDouble());
            }
        };
    }

    /**
     * Returns a command that continuously drives the shooter in coupled mode
     * using a scaled input supplier.
     */
    public Command cmdSetCoupledIPSFactor(DoubleSupplier ipsSupplier, double factor) {
        Objects.requireNonNull(ipsSupplier, "ipsSupplier must not be null");
        return cmdSetCoupledIPS(() -> ipsSupplier.getAsDouble() * factor);
    }

    public double getMainFlywheelSpeedIPS() {
        return mainFlywheel.getSpeedIPS();
    }

    public double getKickerSpeedIPS() {
        return kicker.getSpeedIPS();
    }

    public double getMirroredMainFlywheelSpeedIPS() {
        return mirroredMainFlywheel.getSpeedIPS();
    }

    public double getBackspinFlywheelSpeedIPS() {
        return backspinFlywheel.getSpeedIPS();
    }

    public void recoverIfResetOccurred() {
        mainFlywheel.recoverIfResetOccurred();
        mirroredMainFlywheel.recoverIfResetOccurred();
        kicker.recoverIfResetOccurred();
        backspinFlywheel.recoverIfResetOccurred();
    }

    private static KrakenFlywheelSubsystem createFlywheel(String canBusName, FlywheelConfig config) {
        return new KrakenFlywheelSubsystem(
                config.canId,
                canBusName,
                config.motorName,
                config.flywheelDiameterInches,
                config.gearRatio,
                config.kS,
                config.kV,
                config.kP,
                config.peakCurrent,
                config.motorReversed,
                config.velocityFilterTimeConstantSeconds);
    }
}
