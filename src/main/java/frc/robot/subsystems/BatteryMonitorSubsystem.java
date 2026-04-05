package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryMonitorSubsystem extends SubsystemBase {
    private static final int LOW_BATTERY_DIO_CHANNEL = 0;
    private static final int POWER_DISTRIBUTION_CAN_ID = 13;
    private static final double LOW_BATTERY_THRESHOLD_VOLTS = 12.3;
    private static final double CONTINUOUS_ALARM_THRESHOLD_VOLTS = 11.9;
    private static final double EFFECTIVE_BATTERY_RESET_HYSTERESIS_VOLTS = 0.1;
    private static final double MAX_BATTERY_VOLTAGE_TEST_CURRENT_AMPS = 1.0;
    private static final double MIN_RESISTANCE_TEST_CURRENT_AMPS = 20.0;
    private static final double SAMPLE_PERIOD_SECONDS = 1.0;
    private static final double LOW_BATTERY_DELAY_SECONDS = 0.5;
    private static final double BEEP_ON_SECONDS = 0.1;
    private static final double BEEP_OFF_SECONDS = 0.1;
    private static final double ALARM_REPEAT_DELAY_SECONDS = 5.0;

    private final PowerDistribution powerDistribution =
            new PowerDistribution(POWER_DISTRIBUTION_CAN_ID, PowerDistribution.ModuleType.kRev);
    private final DigitalOutput lowBatteryOutput = new DigitalOutput(LOW_BATTERY_DIO_CHANNEL);
    private final DoublePublisher batteryCurrentPublisher;
    private final DoublePublisher averageInternalResistancePublisher;

    private double effectiveBatteryVoltage = Double.NaN;
    private double latestMeasuredBatteryVoltage = Double.NaN;
    private double latestTotalCurrentAmps = Double.NaN;
    private double lastSampleTimeSeconds = Double.NaN;
    private double accumulatedResistanceVoltageDropVolts = 0.0;
    private double accumulatedResistanceCurrentAmps = 0.0;
    private double lowVoltageStartTimeSeconds = Double.NaN;
    private double alarmCycleStartTimeSeconds = Double.NaN;
    private int alarmCycleBeeps;
    private boolean continuousAlarmMode;

    public BatteryMonitorSubsystem() {
        NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
        batteryCurrentPublisher = tuningTable.getDoubleTopic("BatteryCurrentAmps").publish();
        averageInternalResistancePublisher =
                tuningTable.getDoubleTopic("BatteryAverageInternalResistanceOhms").publish();
        batteryCurrentPublisher.set(0.0);
        averageInternalResistancePublisher.set(0.0);
        lowBatteryOutput.set(false);
        alarmCycleBeeps = 0;
        continuousAlarmMode = false;
    }

    @Override
    public void periodic() {
        double nowSeconds = Timer.getFPGATimestamp();
        if (Double.isNaN(lastSampleTimeSeconds)
                || nowSeconds - lastSampleTimeSeconds >= SAMPLE_PERIOD_SECONDS) {
            sampleBatteryState(nowSeconds);
        }

        if (!Double.isNaN(effectiveBatteryVoltage) && effectiveBatteryVoltage < LOW_BATTERY_THRESHOLD_VOLTS) {
            if (Double.isNaN(lowVoltageStartTimeSeconds)) {
                lowVoltageStartTimeSeconds = nowSeconds;
            }

            if (nowSeconds - lowVoltageStartTimeSeconds < LOW_BATTERY_DELAY_SECONDS) {
                lowBatteryOutput.set(false);
                return;
            }

            if (effectiveBatteryVoltage <= CONTINUOUS_ALARM_THRESHOLD_VOLTS) {
                runContinuousAlarm(nowSeconds);
                return;
            }

            runAlarmCycle(nowSeconds, getRequiredBeeps(effectiveBatteryVoltage));
            return;
        }

        clearAlarmState();
    }

    private void sampleBatteryState(double nowSeconds) {
        lastSampleTimeSeconds = nowSeconds;
        latestMeasuredBatteryVoltage = RobotController.getBatteryVoltage();
        latestTotalCurrentAmps = powerDistribution.getTotalCurrent();

        batteryCurrentPublisher.set(latestTotalCurrentAmps);

        if (latestTotalCurrentAmps <= MAX_BATTERY_VOLTAGE_TEST_CURRENT_AMPS) {
            updateEffectiveBatteryVoltage(latestMeasuredBatteryVoltage);
        }

        if (latestTotalCurrentAmps >= MIN_RESISTANCE_TEST_CURRENT_AMPS) {
            double voltageDropVolts = Math.max(0.0, effectiveBatteryVoltage - latestMeasuredBatteryVoltage);
            accumulatedResistanceVoltageDropVolts += voltageDropVolts;
            accumulatedResistanceCurrentAmps += latestTotalCurrentAmps;
            averageInternalResistancePublisher.set(
                    accumulatedResistanceVoltageDropVolts / accumulatedResistanceCurrentAmps);
        }
    }

    private void updateEffectiveBatteryVoltage(double measuredBatteryVoltage) {
        if (Double.isNaN(effectiveBatteryVoltage)
                || measuredBatteryVoltage < effectiveBatteryVoltage
                || measuredBatteryVoltage
                        >= effectiveBatteryVoltage + EFFECTIVE_BATTERY_RESET_HYSTERESIS_VOLTS) {
            effectiveBatteryVoltage = measuredBatteryVoltage;
        }
    }

    private void runAlarmCycle(double nowSeconds, int requiredBeeps) {
        if (continuousAlarmMode) {
            alarmCycleStartTimeSeconds = Double.NaN;
            continuousAlarmMode = false;
        }

        double sequenceDurationSeconds = alarmCycleBeeps * (BEEP_ON_SECONDS + BEEP_OFF_SECONDS);
        double fullCycleDurationSeconds = sequenceDurationSeconds + ALARM_REPEAT_DELAY_SECONDS;

        if (Double.isNaN(alarmCycleStartTimeSeconds)
                || nowSeconds - alarmCycleStartTimeSeconds >= fullCycleDurationSeconds) {
            alarmCycleStartTimeSeconds = nowSeconds;
            alarmCycleBeeps = requiredBeeps;
            sequenceDurationSeconds = alarmCycleBeeps * (BEEP_ON_SECONDS + BEEP_OFF_SECONDS);
        }

        double elapsedSeconds = nowSeconds - alarmCycleStartTimeSeconds;
        if (elapsedSeconds >= sequenceDurationSeconds) {
            lowBatteryOutput.set(false);
            return;
        }

        double beepWindowSeconds = BEEP_ON_SECONDS + BEEP_OFF_SECONDS;
        double timeIntoCurrentBeepSeconds =
                elapsedSeconds - Math.floor(elapsedSeconds / beepWindowSeconds) * beepWindowSeconds;
        lowBatteryOutput.set(timeIntoCurrentBeepSeconds < BEEP_ON_SECONDS);
    }

    private void runContinuousAlarm(double nowSeconds) {
        if (!continuousAlarmMode || Double.isNaN(alarmCycleStartTimeSeconds)) {
            alarmCycleStartTimeSeconds = nowSeconds;
            continuousAlarmMode = true;
            alarmCycleBeeps = 0;
        }

        double beepWindowSeconds = BEEP_ON_SECONDS + BEEP_OFF_SECONDS;
        double elapsedSeconds = nowSeconds - alarmCycleStartTimeSeconds;
        double timeIntoCurrentBeepSeconds =
                elapsedSeconds - Math.floor(elapsedSeconds / beepWindowSeconds) * beepWindowSeconds;
        lowBatteryOutput.set(timeIntoCurrentBeepSeconds < BEEP_ON_SECONDS);
    }

    private int getRequiredBeeps(double batteryVoltage) {
        return Math.max(1, (int) Math.ceil((LOW_BATTERY_THRESHOLD_VOLTS - batteryVoltage) * 10.0));
    }

    private void clearAlarmState() {
        lowVoltageStartTimeSeconds = Double.NaN;
        alarmCycleStartTimeSeconds = Double.NaN;
        alarmCycleBeeps = 0;
        continuousAlarmMode = false;
        lowBatteryOutput.set(false);
    }
}
