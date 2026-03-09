package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static java.lang.Math.PI;

public class SparkmaxPositionSubsystem extends SubsystemBase {
    private final SparkMax primaryMotor;
    private final SparkMax reverseMotor;

    private final RelativeEncoder primaryEncoder;
    private final RelativeEncoder reverseEncoder;

    private double primaryTargetAngle;
    private double reverseTargetAngle;

    private final PIDController primaryVoltageController;
    private final PIDController reverseVoltageController;

    private double reverseAngleOffset;
    
    private double primaryInput;
    private double reverseInput;

    private double reverse(double theta) {
        return PI * 2 - theta;
    }
    

    public SparkmaxPositionSubsystem(int primaryId, int reverseId) {
        primaryMotor = new SparkMax(primaryId, MotorType.kBrushless);
        reverseMotor = new SparkMax(reverseId, MotorType.kBrushless);


        primaryEncoder = primaryMotor.getEncoder();
        reverseEncoder = reverseMotor.getEncoder();

        reverseAngleOffset = reverse(reverseEncoder.getPosition()) - primaryEncoder.getPosition();

        primaryVoltageController = new PIDController(Constants.MotorConstants.MotorPIDs.PRIMARY_TURRET_K_P, Constants.MotorConstants.MotorPIDs.PRIMARY_TURRET_K_D, Constants.MotorConstants.MotorPIDs.PRIMARY_TURRET_K_I);
        reverseVoltageController = new PIDController(Constants.MotorConstants.MotorPIDs.REVERSE_TURRED_K_P, Constants.MotorConstants.MotorPIDs.REVERSE_TURRED_K_D, Constants.MotorConstants.MotorPIDs.REVERSE_TURRED_K_I);
    }

    public void setTargetAngle(double theta) {
        primaryTargetAngle = theta;
        reverseTargetAngle = reverse(theta) + reverseAngleOffset;
    }

    public void periodic() {
        primaryInput = primaryVoltageController.calculate(primaryEncoder.getPosition(), primaryTargetAngle);
        reverseInput = reverseVoltageController.calculate(reverseEncoder.getPosition(), reverseTargetAngle);

        primaryMotor.setVoltage(primaryInput);
        reverseMotor.setVoltage(reverseInput);
    }
}