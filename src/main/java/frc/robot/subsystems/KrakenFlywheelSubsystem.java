package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;

// For tuning see https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
// Clockwise positive on Left flywheel
// Right flywheel (42) set to follow left, Opposed, ID 41
// kp=6
// ks=2.2
// kv=0.01

public class KrakenFlywheelSubsystem extends SubsystemBase {
    // hardware references
    private TalonFX kraken;

    // command request
    private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0)
                    .withSlot(0)
                    .withOverrideCoastDurNeutral(true);
    
    private final NeutralOut neutralRequest = new NeutralOut();
    // need to set neutralRequest mode to coast

    // flywheel data
    private double flywheelDiameterInches;

    // command inputs
    private DoubleSupplier desiredRPSSupplier; // Commands change this to their own supplier

    // behavior monitoring
    private double desiredRPS;

    public KrakenFlywheelSubsystem(int canId, String motorName, double flywheelDiameterInches, double kS, double kV, double kP, double peakCurrent) {
        this.flywheelDiameterInches = flywheelDiameterInches;
        if (flywheelDiameterInches == 0) throw new IllegalArgumentException ("ASSERTION FAILED:"+
                " flywheelDiameterInches cannot be 0.");
        kraken=new TalonFX(canId);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Slot 0 — VelocityTorqueCurrentFOC gains
        // kS: static friction feedforward (amps)
        // kV: velocity feedforward (amps per RPS)
        // kP: proportional gain (amps per RPS of error)
        cfg.Slot0.kS = kS;
        cfg.Slot0.kV = kV;
        cfg.Slot0.kP = kP;

        // Torque current limits — tune to your mechanism
        cfg.TorqueCurrent.PeakForwardTorqueCurrent  =  peakCurrent; // amps
        cfg.TorqueCurrent.PeakReverseTorqueCurrent  = -peakCurrent; // amps

        // Attempt apply; warn on failure but don't crash the robot
        StatusCode status = kraken.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            DriverStation.reportError(
                    "[" + motorName + "] TalonFX config failed: " + status, false);
        }

        // Optimize CAN bus utilization — only update these signals at the given frequency
        /*BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                kraken.getVelocity()
        );*/
        // Reduce all other signals to 6hz
        kraken.optimizeBusUtilization();
   
        setDefaultCommand(cmdCoast());
    }

    @Override
    public void periodic() {
    }

    public Command cmdCoast() {
        return new OverrideCommand (this){
            @Override
            public void initialize() {
                kraken.setControl(neutralRequest);
            }
        };
    }

    public Command cmdSetRPS(DoubleSupplier rps) {
        return new OverrideCommand (this) {
            @Override
            public void initialize() {
                desiredRPSSupplier = rps;
            }

            @Override
            public void execute() {
                double newDesiredRPS = desiredRPSSupplier.getAsDouble(); 
                if(desiredRPS!=newDesiredRPS) {
                    desiredRPS=newDesiredRPS;
                    kraken.setControl(velocityRequest.withVelocity(desiredRPS));
                }
            }
        };
    }


    public Command cmdSetRPM(DoubleSupplier rpm) {
        return cmdSetRPS(()->rpm.getAsDouble()/60);
    }
    
    public Command cmdSetIPS(DoubleSupplier ips) {
        return cmdSetRPS(() -> ips.getAsDouble() / Math.PI / flywheelDiameterInches);
    }
}


