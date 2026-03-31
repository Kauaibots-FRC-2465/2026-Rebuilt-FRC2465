package frc.robot.Commands;

public final class FlywheelBallExitInterpolator {
    private static final double EPSILON = 1e-9;

    private static final double[] SET_IPS = {
        200.0, 240.0, 280.0, 320.0, 360.0, 400.0, 440.0, 480.0,
        520.0, 560.0, 600.0, 640.0, 680.0, 720.0, 760.0, 800.0
    };

    private static final double[] BALL_EXIT_IPS = {
        143.7223601, 176.6259052, 216.1407088, 272.4331199,
        317.9685292, 355.4835685, 370.3047542, 392.1787301,
        408.0032517, 423.5261656, 438.1693087, 451.8488867,
        469.9891753, 483.3837431, 492.3584267, 507.7218
    };

    static {
        if (SET_IPS.length != BALL_EXIT_IPS.length) {
            throw new IllegalStateException("SET_IPS and BALL_EXIT_IPS must have matching lengths.");
        }
    }

    private FlywheelBallExitInterpolator() {
    }

    public static double getBallExitIpsForSetIps(double setIps) {
        return interpolate(setIps, SET_IPS, BALL_EXIT_IPS);
    }

    public static double getSetIpsForBallExitIps(double ballExitIps) {
        return interpolate(ballExitIps, BALL_EXIT_IPS, SET_IPS);
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
