package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryMonitorSubsystem extends SubsystemBase {
    private static final int LOW_BATTERY_DIO_CHANNEL = 0;
    private static final int TEST_DIO_CHANNEL = 1;
    private static final double LOW_BATTERY_THRESHOLD_VOLTS = 12.0;
    private static final double LOW_BATTERY_DELAY_SECONDS = 0.5;

    private final DigitalOutput lowBatteryOutput = new DigitalOutput(LOW_BATTERY_DIO_CHANNEL);
    private final DigitalOutput testOutput = new DigitalOutput(TEST_DIO_CHANNEL);

    private double lowVoltageStartTimeSeconds = Double.NaN;

    public BatteryMonitorSubsystem() {
        lowBatteryOutput.set(false);
        testOutput.set(true);
    }

    @Override
    public void periodic() {
        testOutput.set(true);

        if (!DriverStation.isDisabled()) {
            resetLowBatteryOutput();
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        double nowSeconds = Timer.getFPGATimestamp();
        if (batteryVoltage < LOW_BATTERY_THRESHOLD_VOLTS) {
            if (Double.isNaN(lowVoltageStartTimeSeconds)) {
                lowVoltageStartTimeSeconds = nowSeconds;
            }

            lowBatteryOutput.set(nowSeconds - lowVoltageStartTimeSeconds >= LOW_BATTERY_DELAY_SECONDS);
            return;
        }

        resetLowBatteryOutput();
    }

    private void resetLowBatteryOutput() {
        lowVoltageStartTimeSeconds = Double.NaN;
        lowBatteryOutput.set(false);
    }
}
