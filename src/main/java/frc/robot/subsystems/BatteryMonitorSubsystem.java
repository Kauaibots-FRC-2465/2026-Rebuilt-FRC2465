package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryMonitorSubsystem extends SubsystemBase {
    private static final int LOW_BATTERY_DIO_CHANNEL = 0;
    private static final double LOW_BATTERY_THRESHOLD_VOLTS = 12.2;
    private static final double CONTINUOUS_ALARM_THRESHOLD_VOLTS = 11.9;
    private static final double EFFECTIVE_BATTERY_RESET_HYSTERESIS_VOLTS = 0.1;
    private static final double LOW_BATTERY_DELAY_SECONDS = 0.5;
    private static final double BEEP_ON_SECONDS = 0.1;
    private static final double BEEP_OFF_SECONDS = 0.1;
    private static final double ALARM_REPEAT_DELAY_SECONDS = 5.0;

    private final DigitalOutput lowBatteryOutput = new DigitalOutput(LOW_BATTERY_DIO_CHANNEL);

    private double effectiveBatteryVoltage = Double.NaN;
    private double lowVoltageStartTimeSeconds = Double.NaN;
    private double alarmCycleStartTimeSeconds = Double.NaN;
    private int alarmCycleBeeps;
    private boolean continuousAlarmMode;

    public BatteryMonitorSubsystem() {
        lowBatteryOutput.set(false);
        alarmCycleBeeps = 0;
        continuousAlarmMode = false;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isDisabled()) {
            resetAlarm();
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        updateEffectiveBatteryVoltage(batteryVoltage);
        double nowSeconds = Timer.getFPGATimestamp();
        if (effectiveBatteryVoltage < LOW_BATTERY_THRESHOLD_VOLTS) {
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

        resetAlarm();
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

    private void resetAlarm() {
        effectiveBatteryVoltage = Double.NaN;
        lowVoltageStartTimeSeconds = Double.NaN;
        alarmCycleStartTimeSeconds = Double.NaN;
        alarmCycleBeeps = 0;
        continuousAlarmMode = false;
        lowBatteryOutput.set(false);
    }
}
