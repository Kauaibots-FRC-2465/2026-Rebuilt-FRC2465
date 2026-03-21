
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KrakenAnglePositionSubsystem;
import frc.robot.subsystems.KrakenFlywheelSubsystem;
import frc.robot.subsystems.KrakenFollowerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PinpointSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.subsystems.SparkFollowerSubsystem;
import frc.robot.subsystems.WLEDSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

// For tuning see https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
// Clockwise positive on Left flywheel
// Right flywheel (42) set to follow left, Opposed, ID 41
// kp=6
// ks=2.2
// kv=0.01

public class RobotContainer implements Subsystem {
    private final static float REMEMBER_TO_SET_CURRENTLY_0 = 0f;
    private final static float REMEMBER_TO_TUNE_CURRENTLY_1 = 1f;
    
    private final static int INCHES = 1;

    private static enum MotorData {
        RIGHT_FLYWHEEL(42, "Right Flywheel Kraken x60"),
        LEFT_FLYWHEEL(41, "Left Flywheel Kraken x60"),
        BACK_FLYWHEEL(44, "Backspin Flywheel Kraken x60"),
        KICKER(40, "Kicker Kraken x60"),
        BACKSPIN(44, "Backspin Kraken x60"),
        LEFT_INTAKE_POSITION(51, "Left Intake Position Kraken x60"),
        RIGHT_INTAKE_POSITION(52, "Right Intake Position Kraken x60"),
        INTAKEDRIVE(53, "Intake Drive Kraken x60"),
        HOOD(3, "Hood Neo 550");

        final int id;
        final String name;

        private MotorData(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 1 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //SparkMax flywheel = new SparkMax(7, MotorType.kBrushless);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driversController = new CommandXboxController(0);
    private final CommandXboxController engineersController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final WLEDSubsystem wled = new WLEDSubsystem();
    private final Command loadLights = wled.loadSingletMarquee(WLEDSubsystem.PATH_1);
    private final Command loadStatic = wled.loadSingletMarquee(WLEDSubsystem.PATH_2);

    public final PoseEstimatorSubsystem poseEstimatorSubsystem;
    public final PoseEstimatorSubsystem.Configuration poseEstimatorConfiguration = new PoseEstimatorSubsystem.Configuration();

    public final SwerveDrivetrainSubsystem encapsulatedDrivetrain;

    public final SparkAnglePositionSubsystem horizontalAimSubsystem;
    public final SparkFollowerSubsystem sparkFollowerSubsystem = new SparkFollowerSubsystem();

    public final SparkAnglePositionSubsystem hood;
    public final KrakenAnglePositionSubsystem leftIntakePosition;
    public final KrakenAnglePositionSubsystem rightIntakePosition;

    public final KrakenFlywheelSubsystem mainShooter;
    public final KrakenFlywheelSubsystem kicker;
    public final KrakenFlywheelSubsystem backspin;
    public final KrakenFlywheelSubsystem intakedrive;


    public final KrakenFollowerSubsystem flywheelFollowerSubsystem = new KrakenFollowerSubsystem("Default Name");
    // CTRE CAN MAP
    // 11 Encoder for Front Left Swerve
    // 12 Encoder for Back Left Swerve
    // 13 Encoder for Back Right Swerve
    // 14 Encoder for Front Right Swerve
    // 21 Kraken x60 Drive of Front Left Swerve
    // 22 Kraken x60 Drive of Back Left Swerve
    // 23 Kraken x60 Drive of Back Right Swerve
    // 24 Kraken x60 Drive of Front Right Swerve
    // 31 Kraken x44 Steer of Front Left Steer
    // 32 Kraken x44 Steer of Back Left Steer
    // 33 Kraken x44 Steer of Back Right Steer
    // 34 Kraken x44 Steer of Front Right Steer
    // 40 Kraken x44 of Kicker
    // 41 Kraken x60 of Left Flywheel
    // 42 Kraken x60 of Right Flywheel
    // 44 Kraken x60 of Backspin Flywheel
    // 50 Kraken x60 of Lifter
    // 51 Kraken x60 of Left Intake
    // 52 Kraken x60 of Right Intake
    // 53 Kraken x60 of Intake Drive

    // ROIBORIO CAN MAP
    // 1  Neo 550 of Left Rotation 
    // 2  Neo 550 of Right Rotation with encoder
    // 3  Neo 550 of Hood Angle

    public RobotContainer() {
        poseEstimatorConfiguration.initialThetaDeviation = Math.toRadians(10.0); 
        poseEstimatorConfiguration.initialXDeviation = 1;
        poseEstimatorConfiguration.initialYDeviation = 1;

        poseEstimatorConfiguration.odoHeadingDeviationPerDistance =
        poseEstimatorConfiguration.odoHeadingDeviationPerRadian =
        poseEstimatorConfiguration.odoLateralDeviationPerDistance =
        poseEstimatorConfiguration.odoLateralDeviationPerRadian =
        poseEstimatorConfiguration.odoLongitudinalDeviationPerDistance =
        poseEstimatorConfiguration.odoLongitudinalDeviationPerRadian = 0.01;

        PinpointSubsystem pinpointSubsystem = new PinpointSubsystem(
            -6.0625 /*y of x-forward pod*/,
            -2.5 /*x of y- strafe pod*/,
            Inch,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.EncoderDirection.FORWARD,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.EncoderDirection.REVERSED);
        poseEstimatorConfiguration.odometryPose = pinpointSubsystem.getPose2dSupplier();
        poseEstimatorConfiguration.odometryTimestamp = pinpointSubsystem.getTimestampSupplier();
        poseEstimatorConfiguration.odometryValid = pinpointSubsystem.getIsValidSupplier();
        
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(pinpointSubsystem.getHeadingSupplier(Degree), false, new int[] 
            {     1,  2,  3,  4,  5,  6,  7,  8,  9,
             10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
             20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
             30, 31, 32});
        poseEstimatorConfiguration.visionPose = limelightSubsystem.getPose2dSupplier();
        poseEstimatorConfiguration.visionTimestamp = limelightSubsystem.getPose2dTimestampSupplier();
        poseEstimatorConfiguration.visionIsValid = limelightSubsystem.getIsValidSupplier();

        poseEstimatorConfiguration.visionThetaDeviation = limelightSubsystem.getThetaDeviationSupplier();
        poseEstimatorConfiguration.visionXDeviation = limelightSubsystem.getXDeviationSupplier();
        poseEstimatorConfiguration.visionYDeviation = limelightSubsystem.getYDeviationSupplier();
        
        poseEstimatorSubsystem = new PoseEstimatorSubsystem(poseEstimatorConfiguration);
        encapsulatedDrivetrain = new SwerveDrivetrainSubsystem(drivetrain, poseEstimatorSubsystem); 

        horizontalAimSubsystem = new SparkAnglePositionSubsystem(
            2,
            "Right Horizontal Aim Neo 550",
            233.0/12.0*5.0, // Positioning gear plate hsa 233 teeth, pinions have 12
            5,
            .2,
            1,
            10,
            40,
            12,
            Degrees.of(-22),
            Degrees.of(22),
            false,
            true);
        sparkFollowerSubsystem.addFollower(
            1,
            "Left Horizontal Aim Neo 550",
            2,
            MotorType.kBrushless,
            10,
            40,
            12,
            IdleMode.kBrake,
            false);

        hood = new SparkAnglePositionSubsystem(MotorData.HOOD.id,
            MotorData.HOOD.name,
            150.0/10.0*22.0/32.0*5.0*5.0,
            5.0*5.0,
            0.06,
            0.0,
            10,
            40,
            12.0,
            Degrees.of(0.0),
            Degrees.of(48.7409948542),
            true,
            false);

        leftIntakePosition = new KrakenAnglePositionSubsystem(
            MotorData.LEFT_INTAKE_POSITION.id,
            "Default Name",
            MotorData.LEFT_INTAKE_POSITION.name,
            47.0 / 10.0 * 3.0,
            0.603027,
            4, //0.5
            40,
            2.0,
            Degrees.of(0.0),
            Degrees.of(110.0),
            true);

        rightIntakePosition = new KrakenAnglePositionSubsystem(
            MotorData.RIGHT_INTAKE_POSITION.id,
            "Default Name",
            MotorData.RIGHT_INTAKE_POSITION.name,
            47.0 / 10.0 * 3.0,
            0.914551,
            4,
            40,
            2.0,
            Degrees.of(0.0),
            Degrees.of(110.0),
            false);


        mainShooter = new KrakenFlywheelSubsystem(
          MotorData.RIGHT_FLYWHEEL.id,   // | Motor ID
          "Default Name",                // | CANivore bus name
          MotorData.RIGHT_FLYWHEEL.name, // | Motor Name
          4 * INCHES,                    // | Wheel size
          16f/24f,                       // | Gear ratio
          2.3,                        // | kS
          0.14,                       // | kV 
          2,                          // | kP
          40                 // | Peak current
        );

        kicker = new KrakenFlywheelSubsystem(
          MotorData.KICKER.id,   // | Motor ID
          "Default Name",                // | CANivore bus name
          MotorData.KICKER.name, // | Motor Name
          3 * INCHES,                    // | Wheel size
          16f/24f,                       // | Gear ratio
          2.7,                        // | kS
          0.0095,                       // | kV 
          2,                          // | kP
          40                 // | Peak current
        );
        
        backspin = new KrakenFlywheelSubsystem(
          MotorData.BACKSPIN.id,   // | Motor ID
          "Default Name",                // | CANivore bus name
          MotorData.BACKSPIN.name, // | Motor Name
          2 * INCHES,                    // | Wheel size
          16f/24f,                       // | Gear ratio
          2.3,                        // | kS
          0.013,                       // | kV 
          2,                          // | kP
          40                 // | Peak current
        );

        intakedrive = new KrakenFlywheelSubsystem(
          MotorData.INTAKEDRIVE.id,   // | Motor ID
          "Default Name",                // | CANivore bus name
          MotorData.INTAKEDRIVE.name, // | Motor Name
          2.5 * INCHES,                    // | Wheel size
          47.0 / 10.0 * 34.0 / 44.0,     // | Gear ratio
          2.3,                        // | kS
          0.013,                       // | kV 
          2,                          // | kP
          40                 // | Peak current
        );
  
        flywheelFollowerSubsystem.addFollower(MotorData.LEFT_FLYWHEEL.id, MotorData.LEFT_FLYWHEEL.name, MotorData.RIGHT_FLYWHEEL.id, 40, MotorAlignmentValue.Opposed, NeutralModeValue.Coast);

        RobotConfig config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        AutoBuilder.configure(
            poseEstimatorSubsystem.getFusedPoseSupplier(), // Robot pose supplier
            (Pose2d $) -> {}, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> {return drivetrain.getState().Speeds;}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) -> drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(chassisSpeeds.vxMetersPerSecond).withVelocityY(chassisSpeeds.vyMetersPerSecond)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


        configureBindings();

    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        schedule(loadLights);
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driversController.getLeftY() /* Math.abs(joystick.getLeftY())*/ * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driversController.getLeftX() /* Math.abs(joystick.getLeftX())*/ * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driversController.getRightX() /* Math.abs(joystick.getRightX())*/ * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        mainShooter.setDefaultCommand(mainShooter.cmdSetIPSFactor(this::getShooterPower, 2000.0));
        kicker.setDefaultCommand(kicker.cmdSetIPSFactor(this::getShooterPower, -2000.0));
        backspin.setDefaultCommand(backspin.cmdSetIPSFactor(this::getShooterPower, 2000.0));

        leftIntakePosition.setDefaultCommand(
            leftIntakePosition.cmdSetAngle(() -> Degrees.of(engineersController.a().getAsBoolean() ? 102.5 : 5.0)));
        rightIntakePosition.setDefaultCommand(
            rightIntakePosition.cmdSetAngle(() -> Degrees.of(engineersController.a().getAsBoolean() ? 102.5 : 5.0))); //97?
        intakedrive.setDefaultCommand(intakedrive.cmdSetIPS(()->engineersController.a().getAsBoolean() ? 125.0 : 0.0)); //600 max

        hood.setDefaultCommand(hood.cmdSetScaledAngle(engineersController::getLeftTriggerAxis));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driversController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driversController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driversController.getLeftY(), -driversController.getLeftX()))
        ));

        driversController.x().onTrue(loadLights);
        driversController.x().onFalse(loadStatic);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driversController.back().and(driversController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driversController.back().and(driversController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driversController.start().and(driversController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driversController.start().and(driversController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

       //*  Reset the field-centric heading on left bumper press.
        driversController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        horizontalAimSubsystem.setDefaultCommand(
            horizontalAimSubsystem.cmdSetScaledAngle(() -> (engineersController.getLeftX() + 1.0) / 2.0));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Plow");
    }

    private void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    private double getShooterPower() {
        final double minPower = 0.25;
        double shooterPower = engineersController.getRightTriggerAxis();
        return (shooterPower < 0.005) ? 0.0 : (shooterPower + minPower) / (1.0 + minPower);
    }
}
