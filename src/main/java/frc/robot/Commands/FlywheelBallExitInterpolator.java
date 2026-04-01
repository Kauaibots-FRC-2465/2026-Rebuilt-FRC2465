package frc.robot.Commands;

public final class FlywheelBallExitInterpolator {
    private static final double EPSILON = 1e-9;

    static {
        if (ShooterConstants.FLYWHEEL_SET_IPS.length != ShooterConstants.BALL_EXIT_IPS.length) {
            throw new IllegalStateException("SET_IPS and BALL_EXIT_IPS must have matching lengths.");
        }
    }

    private FlywheelBallExitInterpolator() {
    }

    public static double getBallExitIpsForSetIps(double setIps) {
        return interpolate(setIps, ShooterConstants.FLYWHEEL_SET_IPS, ShooterConstants.BALL_EXIT_IPS);
    }

    public static double getSetIpsForBallExitIps(double ballExitIps) {
        return interpolate(ballExitIps, ShooterConstants.BALL_EXIT_IPS, ShooterConstants.FLYWHEEL_SET_IPS);
    }

    private static double interpolate(double input, double[] inputs, double[] outputs) {
        if (!Double.isFinite(input)) {
            return Double.NaN;
        }

        for (int i = 0; i < inputs.length; i++) {
            if (Math.abs(input - inputs[i]) <= EPSILON) {
                return outputs[i];
            }
        }

        if (input < inputs[0] || input > inputs[inputs.length - 1]) {
            return Double.NaN;
        }

        for (int i = 0; i < inputs.length - 1; i++) {
            double input1 = inputs[i];
            double input2 = inputs[i + 1];
            if (input >= input1 && input <= input2) {
                double output1 = outputs[i];
                double output2 = outputs[i + 1];
                return output1 + (output2 - output1) * ((input - input1) / (input2 - input1));
            }
        }

        return Double.NaN;
    }
}
