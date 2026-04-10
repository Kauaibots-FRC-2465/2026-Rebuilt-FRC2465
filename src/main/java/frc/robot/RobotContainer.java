
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.json.simple.parser.ParseException;

import frc.robot.Commands.ScoreInHub;
import frc.robot.Commands.SnowblowToAlliance;
import frc.robot.Commands.SnowblowToAllianceWithOperatorAim;
import frc.robot.Commands.Rebound;
import frc.robot.Commands.DataCollectionCommand;
import frc.robot.Commands.HoodRestPositionCharacterization;
import frc.robot.Commands.HoodTimingCharacterization;
import frc.robot.Commands.PathPlannerAutoAssist;
import frc.robot.Commands.ShooterConstants;
import frc.robot.Commands.TestShootingCommand;
import frc.robot.fieldmath.FieldMath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.BatteryMonitorSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.KrakenFlywheelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PinpointSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.subsystems.SparkFollowerSubsystem;
import frc.robot.subsystems.WLEDSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.utility.GameplayDashboard;

// For tuning see https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
// Clockwise positive on Left flywheel
// Main flywheels are commanded independently; left is inverted relative to right
// kp=6
// ks=2.2
// kv=0.01

// SYSID for this robot
// 0.1435  Ks straight
// 0.1166  Kv straight
// 0.01932 Ka sttraight

// 0.19858 Ks rotation
// 0.11599 Kv rotation
// 0.021855 Ka rotation

public class RobotContainer implements Subsystem {
    private final static float REMEMBER_TO_SET_CURRENTLY_0 = 0f;
    private final static float REMEMBER_TO_TUNE_CURRENTLY_1 = 1f;
    
    private final static int INCHES = 1;
    private static final double HOOD_TUNE_ANGLE_STEP_DEGREES = 5.0;
    private static final double SHOOTER_TUNE_SPEED_STEP_IPS = 40;
    private static final double HORIZONTAL_AIM_TRIM_STEP_DEGREES = 1.0;
    // Intake trim needs a larger step so the Kraken position loop can overcome
    // mechanism load and produce a visible change per D-pad press.
    private static final double ACTIVE_INTAKE_ANGLE_STEP_DEGREES = 0.5;
    private static final double DEFAULT_ACTIVE_INTAKE_ANGLE_DEGREES = 109.0;
    private static final double MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS = 0.010;
    private static final double DRIVE_TUNING_LOW_SPEED_METERS_PER_SECOND = 1.0;
    private static final double DRIVE_TUNING_HIGH_SPEED_METERS_PER_SECOND = 2.0;
    private static final double DRIVE_TUNING_ROTATION_RADIANS_PER_SECOND = Math.PI / 2.0;

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
            .withDriveRequestType(DriveRequestType.Velocity); // Use velocity closed-loop for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric driveTuningRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
    private final DoublePublisher hoodTuneAnglePublisher = tuningTable.getDoubleTopic("HoodTuneAngleDegrees").publish();
    private final DoublePublisher shooterTuneSpeedPublisher = tuningTable.getDoubleTopic("ShooterTuneSpeedIps").publish();
    private final DoublePublisher mainFlywheelMeasuredSpeedPublisher =
            tuningTable.getDoubleTopic("MainFlywheelMeasuredSpeedIps").publish();

    private final CommandXboxController driversController = new CommandXboxController(0);
    private final CommandXboxController engineersController = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(2);
    private final CommandXboxController testController2 = new CommandXboxController(3);
    private final CommandXboxController driveTuningController = new CommandXboxController(4);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final BatteryMonitorSubsystem batteryMontior = new BatteryMonitorSubsystem();
    public final WLEDSubsystem wled = new WLEDSubsystem();
    private final Command showLights = wled.showMarquee( Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Sprite-0001.bmp");
    private final Command showStatic = wled.showMarquee(Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Sprite-0002.bmp");
    private final Command show2465 = wled.showMarquee(Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "2465.bmp");
    private final Command showBamPowZang = wled.showGIF(Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "bampowzang5.gif");

    private final String redAllianceMarqueePath =
    Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "red.bmp";
    private final String blueAllianceMarqueePath =
    Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "blue.bmp";
    private final WLEDSubsystem.PreparedPlaybackPackets redAlliancePlayback =
    wled.prepareMarquee(redAllianceMarqueePath);
    private final WLEDSubsystem.PreparedPlaybackPackets blueAlliancePlayback =
    wled.prepareMarquee(blueAllianceMarqueePath);


private Command showAllianceMarquee() {
    return new OverrideCommand(wled) {
        private DriverStation.Alliance lastAlliance;

        @Override
        public void initialize() {
            lastAlliance = null;
        }

        @Override
        public void execute() {
            DriverStation.Alliance currentAlliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

            if (currentAlliance == lastAlliance) {
                return;
            }

            wled.setActivePlayback(
                currentAlliance == DriverStation.Alliance.Red
                    ? redAlliancePlayback
                    : blueAlliancePlayback
            );

            lastAlliance = currentAlliance;
        }
    }.ignoringDisable(true);
}



    public final PoseEstimatorSubsystem poseEstimatorSubsystem;
    public final PoseEstimatorSubsystem.Configuration poseEstimatorConfiguration = new PoseEstimatorSubsystem.Configuration();
    private final PinpointSubsystem pinpointSubsystem;
    private boolean hasReliableFieldHeading = false;

    public final SwerveDrivetrainSubsystem encapsulatedDrivetrain;

    public final SparkAnglePositionSubsystem horizontalAim;
    public final SparkFollowerSubsystem sparkFollowerSubsystem = new SparkFollowerSubsystem();

    public final SparkAnglePositionSubsystem verticalAim;
    public final IntakePositionSubsystem intakePosition;

    public final ShooterSubsystem shooter;
    public final KrakenFlywheelSubsystem intakedrive;
    private final DataCollectionCommand dataCollectionCommand;
    private final TestShootingCommand testShootingCommand;
    private final ScoreInHub scoreInHubCommand;
    private final SnowblowToAllianceWithOperatorAim snowblowToAllianceWithOperatorAimCommand;
    private final PathPlannerAutoAssist pathPlannerAutoAssist;
    private final SendableChooser<Command> autoChooser;
    private final Map<String, PointTowardsZoneTrigger> pointTowardsZoneTriggers = new HashMap<>();
    private final Map<String, PathPlannerPath> preloadedPointTowardsPaths = new HashMap<>();
    private final Map<String, PathPlannerPath> preloadedFlippedPointTowardsPaths = new HashMap<>();
    private final Set<String> failedPointTowardsPaths = new HashSet<>();
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

        pinpointSubsystem = new PinpointSubsystem(
            -6.0625 /*y of x-forward pod*/,
            -2.5 /*x of y- strafe pod*/,
            Inch,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.EncoderDirection.FORWARD,
            frc.robot.subsystems.GoBildaPinpointFRCDriver.EncoderDirection.REVERSED);
        poseEstimatorConfiguration.odometryPose = pinpointSubsystem.getPose2dSupplier();
        poseEstimatorConfiguration.odometryTimestamp = pinpointSubsystem.getTimestampSupplier();
        poseEstimatorConfiguration.odometryValid = pinpointSubsystem.getIsValidSupplier();
        
        DoubleSupplier limelightHeadingDegreesSupplier = () -> {
            Pose2d odometryPose = this.pinpointSubsystem.getPose2dSupplier().get();
            if (odometryPose != null) {
                return odometryPose.getRotation().getDegrees();
            }
            return this.pinpointSubsystem.getHeadingSupplier(Degree).getAsDouble();
        };
        BooleanSupplier limelightHeadingReliableSupplier = () -> hasReliableFieldHeading;
        Consumer<Pose2d> limelightMt1SeedConsumer = this::seedPoseFromMt1;
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(
            limelightHeadingDegreesSupplier,
            limelightHeadingReliableSupplier,
            limelightMt1SeedConsumer,
            true,
            new int[] 
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

        horizontalAim = new SparkAnglePositionSubsystem(
            2,
            "Right Horizontal Aim Neo 550",
            233.0/12.0*5.0, // Positioning gear plate hsa 233 teeth, pinions have 12
            5,
            .2,
            1,
            10,
            40,
            12,
            Degrees.of(-18),
            Degrees.of(18),
            true,
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
        horizontalAim.enableAbsoluteDriftMonitor("HorizontalAim");

/*        hood = new SparkAnglePositionSubsystem(MotorData.HOOD.id,
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
            false);*/
        verticalAim = new SparkAnglePositionSubsystem(MotorData.HOOD.id,
            MotorData.HOOD.name,
            150.0/10.0*22.0/32.0*5.0*5.0,
            5.0*5.0,
            .06,//.2, //0.06,
            0.0,
            10,
            40,
            12.0,
            Degrees.of(ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES),
            Degrees.of(ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES),
            Degrees.of(ShooterConstants.MEASURED_HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES),
            false,
            true,
            true,
            false,
            false);

        intakePosition = new IntakePositionSubsystem(
            MotorData.LEFT_INTAKE_POSITION.id,
            MotorData.LEFT_INTAKE_POSITION.name,
            0.603027,
            true,
            MotorData.RIGHT_INTAKE_POSITION.id,
            MotorData.RIGHT_INTAKE_POSITION.name,
            0.914551,
            false,
            "Default Name",
            47.0 / 10.0 * 3.0,
            4,
            40,
            2.0,
            Degrees.of(0.0),
            Degrees.of(113.0));


        shooter = new ShooterSubsystem(
          "Default Name",
          new ShooterSubsystem.FlywheelConfig(
            MotorData.RIGHT_FLYWHEEL.id,
            MotorData.RIGHT_FLYWHEEL.name,
            4 * INCHES,
            24f/16f,
            2.3,
            0,
            2,
            30,
            false,
            MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS),
          new ShooterSubsystem.FlywheelConfig(
            MotorData.LEFT_FLYWHEEL.id,
            MotorData.LEFT_FLYWHEEL.name,
            4 * INCHES,
            24f/16f,
            2.3,
            0,
            2,
            30,
            true,
            MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS),
          new ShooterSubsystem.FlywheelConfig(
            MotorData.KICKER.id,
            MotorData.KICKER.name,
            3 * INCHES,
            24f/16f,
            2.7,
            0.0095,
            2,
            30,
            true,
            MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS),
          new ShooterSubsystem.FlywheelConfig(
            MotorData.BACKSPIN.id,
            MotorData.BACKSPIN.name,
            2 * INCHES,
            24f/16f,
            2.3,
            0.013,
            2,
            30)
        );

        intakedrive = new KrakenFlywheelSubsystem(
          MotorData.INTAKEDRIVE.id,   // | Motor ID
          "Default Name",                // | CANivore bus name
          MotorData.INTAKEDRIVE.name, // | Motor Name
          2.5 * INCHES,                    // | Wheel size
          47.0 / 44.0,     // | Gear ratio 44 intake, 66 idler, 47 motor
          2.3,                        // | kS
          0.013,                       // | kV 
          2,                          // | kP
          20,                 // | Peak current
          false,
          MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS
        );
        dataCollectionCommand = new DataCollectionCommand(
                poseEstimatorSubsystem,
                horizontalAim,
                verticalAim,
                intakePosition,
                shooter,
                testController);
        testShootingCommand = new TestShootingCommand(
            drivetrain,
            poseEstimatorSubsystem,
            horizontalAim,
            verticalAim,
            intakePosition,
            shooter,
            intakedrive,
            this::getDriverDriveRequest);
        scoreInHubCommand = new ScoreInHub(
            drivetrain,
            pinpointSubsystem,
            poseEstimatorSubsystem,
            horizontalAim,
            verticalAim,
            shooter,
            this::getDriverDriveRequest);
        snowblowToAllianceWithOperatorAimCommand = new SnowblowToAllianceWithOperatorAim(
            drivetrain,
            poseEstimatorSubsystem,
            horizontalAim,
            verticalAim,
            shooter,
            this::getDriverDriveRequest,
            this::getEngineersTarget);
        preloadPointTowardsPaths();
        pathPlannerAutoAssist = new PathPlannerAutoAssist(
                poseEstimatorSubsystem,
                horizontalAim,
                verticalAim,
                shooter,
                this::getActivePointTowardsTarget);
        RobotConfig config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPathPlannerPose, // Robot pose supplier
            this::resetAutonomousPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> {return drivetrain.getState().Speeds;}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::applyPathPlannerDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
            drivetrain // Reference to the drivetrain so auto interrupts the default drive command
    );

        configurePathPlannerBindings();
        autoChooser = buildAutoChooser();
        GameplayDashboard.publishSendable("AutoChooser", autoChooser);
        //hi :D
        configureBindings();
        // Debug tuning telemetry disabled to reduce NetworkTables traffic.
        // publishTuningTelemetry();

    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        //schedule(showLights);
        wled.setDefaultCommand(showAllianceMarquee());
        //schedule(show2465);
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(this::getDriverDriveRequest)
        );

        //shooter.setDefaultCommand(shooter.cmdSetCoupledIPSFactor(this::getShooterPower, 1500.0));
        shooter.setDefaultCommand(shooter.cmdSetCoupledIPS(this::getDesiredShooterIps));

        intakePosition.setDefaultCommand(intakePosition.cmdSetAngle(this::getDesiredIntakeAngle));
        intakedrive.setDefaultCommand(intakedrive.cmdSetIPS(this::getDesiredIntakeDriveIps)); //600 max

        //hood.setDefaultCommand(hood.cmdSetScaledAngle(engineersController::getLeftTriggerAxis));
        verticalAim.setDefaultCommand(verticalAim.cmdSetAngle(this::getDesiredHoodAngle));
        testController.povUp().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(testShootingCommand)) {
                testShootingCommand.trimHoodUp();
            } else {
                adjustHoodTuneAngle(HOOD_TUNE_ANGLE_STEP_DEGREES);
            }
        }));
        testController.povDown().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(testShootingCommand)) {
                testShootingCommand.trimHoodDown();
            } else {
                adjustHoodTuneAngle(-HOOD_TUNE_ANGLE_STEP_DEGREES);
            }
        }));
        engineersController.povRight().onTrue(Commands.runOnce(() -> {
            horizontalAim.adjustAngleTrim(Degrees.of(-HORIZONTAL_AIM_TRIM_STEP_DEGREES));
        }));
        engineersController.povLeft().onTrue(Commands.runOnce(() -> {
            horizontalAim.adjustAngleTrim(Degrees.of(HORIZONTAL_AIM_TRIM_STEP_DEGREES));
        }));
        engineersController.povUp().onTrue(Commands.runOnce(() -> {
            adjustActiveIntakeAngle(-ACTIVE_INTAKE_ANGLE_STEP_DEGREES);
        }));
        engineersController.povDown().onTrue(Commands.runOnce(() -> {
            adjustActiveIntakeAngle(ACTIVE_INTAKE_ANGLE_STEP_DEGREES);
        }));
        testController.povRight().onTrue(Commands.runOnce(() -> {
            if (!CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                adjustShooterTuneSpeed(SHOOTER_TUNE_SPEED_STEP_IPS);
            }
        }));
        testController.povLeft().onTrue(Commands.runOnce(() -> {
            if (!CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                adjustShooterTuneSpeed(-SHOOTER_TUNE_SPEED_STEP_IPS);
            }
        }));
        testController.x().toggleOnTrue(testShootingCommand);
        testController.y().toggleOnTrue(dataCollectionCommand);
        engineersController.start().onTrue(Commands.runOnce(() -> {
            shooter.recoverIfResetOccurred();
            intakedrive.recoverIfResetOccurred();
            intakePosition.recoverIfResetOccurred();
        }));
        engineersController.rightTrigger().whileTrue(
            new Rebound(
                horizontalAim,
                verticalAim,
                shooter,
                intakePosition,
                intakedrive)
        );
        engineersController.leftBumper().onTrue(
            Commands.runOnce(this::resetFieldOrientedHeadingToAllianceForward)
                    .ignoringDisable(true)
        );
        testController2.y().onTrue(new HoodRestPositionCharacterization(verticalAim));
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
        driversController.povUp().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                dataCollectionCommand.adjustTargetDistanceUp();
            }
        }));
        driversController.povDown().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                dataCollectionCommand.adjustTargetDistanceDown();
            }
        }));
        driversController.povLeft().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                dataCollectionCommand.adjustTargetAzimuthLeft();
            }
        }));
        driversController.povRight().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                dataCollectionCommand.adjustTargetAzimuthRight();
            }
        }));
        driversController.y().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
                dataCollectionCommand.refreshTargetDistanceFromPoseSample();
            }
        }));

        driversController.rightTrigger().whileTrue(
            snowblowToAllianceWithOperatorAimCommand
        );
        driversController.leftTrigger().whileTrue(
            scoreInHubCommand
        );

        driveTuningController.a().whileTrue(createDriveTuningCommand(
                DRIVE_TUNING_LOW_SPEED_METERS_PER_SECOND,
                0.0,
                0.0));
        driveTuningController.b().whileTrue(createDriveTuningCommand(
                -DRIVE_TUNING_LOW_SPEED_METERS_PER_SECOND,
                0.0,
                0.0));
        driveTuningController.x().whileTrue(createDriveTuningCommand(
                0.0,
                DRIVE_TUNING_LOW_SPEED_METERS_PER_SECOND,
                0.0));
        driveTuningController.y().whileTrue(createDriveTuningCommand(
                0.0,
                -DRIVE_TUNING_LOW_SPEED_METERS_PER_SECOND,
                0.0));
        driveTuningController.leftBumper().whileTrue(createDriveTuningCommand(
                0.0,
                0.0,
                DRIVE_TUNING_ROTATION_RADIANS_PER_SECOND));
        driveTuningController.rightBumper().whileTrue(createDriveTuningCommand(
                0.0,
                0.0,
                -DRIVE_TUNING_ROTATION_RADIANS_PER_SECOND));
        driveTuningController.povUp().whileTrue(createDriveTuningCommand(
                DRIVE_TUNING_HIGH_SPEED_METERS_PER_SECOND,
                0.0,
                0.0));
        driveTuningController.povDown().whileTrue(createDriveTuningCommand(
                -DRIVE_TUNING_HIGH_SPEED_METERS_PER_SECOND,
                0.0,
                0.0));
        driveTuningController.povLeft().whileTrue(createDriveTuningCommand(
                0.0,
                DRIVE_TUNING_HIGH_SPEED_METERS_PER_SECOND,
                0.0));
        driveTuningController.povRight().whileTrue(createDriveTuningCommand(
                0.0,
                -DRIVE_TUNING_HIGH_SPEED_METERS_PER_SECOND,
                0.0));
        driveTuningController.start().and(driveTuningController.y()).whileTrue(
                drivetrain.sysIdQuasistatic(Direction.kForward));
        driveTuningController.start().and(driveTuningController.x()).whileTrue(
                drivetrain.sysIdQuasistatic(Direction.kReverse));
        driveTuningController.back().and(driveTuningController.y()).whileTrue(
                drivetrain.sysIdDynamic(Direction.kForward));
        driveTuningController.back().and(driveTuningController.x()).whileTrue(
                drivetrain.sysIdDynamic(Direction.kReverse));
        driveTuningController.leftTrigger().and(driveTuningController.start()).whileTrue(
                drivetrain.sysIdRotationQuasistatic(Direction.kForward));
        driveTuningController.leftTrigger().and(driveTuningController.back()).whileTrue(
                drivetrain.sysIdRotationQuasistatic(Direction.kReverse));
        driveTuningController.rightTrigger().and(driveTuningController.start()).whileTrue(
                drivetrain.sysIdRotationDynamic(Direction.kForward));
        driveTuningController.rightTrigger().and(driveTuningController.back()).whileTrue(
                drivetrain.sysIdRotationDynamic(Direction.kReverse));

       //*  Seed the driver-perspective offset from the fused heading on left bumper press.
        driversController.leftBumper().onTrue(
            Commands.runOnce(() -> {
                Pose2d fusedPose = poseEstimatorSubsystem.getFusedPoseSupplier().get();
                if (fusedPose != null) {
                    drivetrain.seedDriverPerspectiveToHeading(fusedPose.getRotation());
                }
            })
        );

        horizontalAim.setDefaultCommand(horizontalAim.cmdSetAngle(this::getDesiredHorizontalAimAngle));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configurePathPlannerBindings() {
        NamedCommands.registerCommand(
                "EnableSnowblowToAlliance",
                Commands.runOnce(pathPlannerAutoAssist::enableSnowblowToAlliance));
        NamedCommands.registerCommand(
                "DisableSnowblowToAlliance",
                Commands.runOnce(pathPlannerAutoAssist::disableShotAssist));
        NamedCommands.registerCommand(
                "EnableScoreInHub",
                Commands.runOnce(pathPlannerAutoAssist::enableScoreInHub));
        NamedCommands.registerCommand(
                "DisableScoreInHub",
                Commands.runOnce(pathPlannerAutoAssist::disableShotAssist));
        NamedCommands.registerCommand(
                "EnableIntake",
                Commands.runOnce(pathPlannerAutoAssist::activateIntake));
        NamedCommands.registerCommand(
                "DisableIntake",
                Commands.runOnce(pathPlannerAutoAssist::deactivateIntake));

        new EventTrigger("SnowblowToAlliance")
                .onTrue(Commands.runOnce(pathPlannerAutoAssist::enableSnowblowToAlliance));
        new EventTrigger("SnowblowToAlliance")
                .onFalse(Commands.runOnce(pathPlannerAutoAssist::disableShotAssist));
        new EventTrigger("ScoreInHub")
                .onTrue(Commands.runOnce(pathPlannerAutoAssist::enableScoreInHub));
        new EventTrigger("ScoreInHub")
                .onFalse(Commands.runOnce(pathPlannerAutoAssist::disableShotAssist));
        new EventTrigger("Intake")
                .onTrue(Commands.runOnce(pathPlannerAutoAssist::activateIntake));
        new EventTrigger("Intake")
                .onFalse(Commands.runOnce(pathPlannerAutoAssist::deactivateIntake));
    }

    private void resetAutonomousPose(Pose2d pose) {
        applyPoseReset(pose, true);
    }

    private void resetFieldOrientedHeadingToAllianceForward() {
        Pose2d currentPose = poseEstimatorSubsystem.getFusedPoseSupplier().get();
        if (currentPose == null) {
            currentPose = pinpointSubsystem.getPose2dSupplier().get();
        }
        if (currentPose == null) {
            return;
        }

        Rotation2d allianceForward = DriverStation.getAlliance()
                .map(allianceColor -> allianceColor == DriverStation.Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero)
                .orElse(Rotation2d.kZero);
        Pose2d resetPose = new Pose2d(currentPose.getTranslation(), allianceForward);
        applyPoseReset(resetPose, true);
    }

    private void seedPoseFromMt1(Pose2d pose) {
        applyPoseReset(pose, true);
    }

    private void applyPoseReset(Pose2d pose, boolean headingReliable) {
        pinpointSubsystem.setPosition(pose);
        poseEstimatorSubsystem.resetPose(pose);
        drivetrain.resetPose(pose);
        hasReliableFieldHeading = headingReliable;
    }

    private void applyPathPlannerDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        drivetrain.setControl(
                pathPlannerAutoAssist.buildDriveRequest(
                        drivetrain,
                        chassisSpeeds,
                        feedforwards));
    }

    public void resetPathPlannerAutoAssist() {
        pathPlannerAutoAssist.reset();
    }

    public Command getAutonomousCommand() {
        Command selectedAuto = autoChooser.getSelected();
        return Commands.sequence(
                Commands.runOnce(this::resetPathPlannerAutoAssist),
                selectedAuto != null ? selectedAuto : Commands.none());
    }

    private void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    private double getShooterPower() {
        final double minPower = 0.4;
        double shooterPower = engineersController.getRightTriggerAxis();
        return (shooterPower < 0.005) ? 0.0 : (shooterPower + minPower) / (1.0 + minPower);
    }

    private Translation2d getEngineersTarget() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return FieldMath.getEngineerTargetInAllianceZone(
                alliance,
                engineersController.getLeftX(),
                engineersController.getLeftY());
    }

    private Pose2d getPathPlannerPose() {
        return drivetrain.getState().Pose;
    }

    private Translation2d getActivePointTowardsTarget() {
        String currentPathName = PathPlannerAuto.currentPathName;
        if (currentPathName == null || currentPathName.isBlank()) {
            return null;
        }

        PathPlannerPath path = getActivePreloadedPointTowardsPath(currentPathName);
        if (path == null) {
            if (failedPointTowardsPaths.add(currentPathName)) {
                DriverStation.reportWarning(
                        "Point towards zones were not preloaded for path: " + currentPathName,
                        new StackTraceElement[0]);
            }
            return null;
        }

        for (var zone : path.getPointTowardsZones()) {
            PointTowardsZoneTrigger trigger = getPointTowardsZoneTrigger(zone.name());
            if (trigger != null && trigger.getAsBoolean()) {
                return zone.targetPosition();
            }
        }

        return null;
    }

    private PointTowardsZoneTrigger getPointTowardsZoneTrigger(String zoneName) {
        return pointTowardsZoneTriggers.get(zoneName);
    }

    private void preloadPointTowardsPaths() {
        File pathDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        File[] pathFiles = pathDirectory.listFiles((dir, name) -> name.endsWith(".path"));
        if (pathFiles == null || pathFiles.length == 0) {
            return;
        }

        Arrays.sort(pathFiles, Comparator.comparing(File::getName, String.CASE_INSENSITIVE_ORDER));

        for (File pathFile : pathFiles) {
            String fileName = pathFile.getName();
            String pathName = fileName.substring(0, fileName.lastIndexOf('.'));
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                PathPlannerPath flippedPath = path.preventFlipping ? path : path.flipPath();
                preloadedPointTowardsPaths.put(pathName, path);
                preloadedFlippedPointTowardsPaths.put(pathName, flippedPath);
                preloadPointTowardsZoneTriggers(path);
                preloadPointTowardsZoneTriggers(flippedPath);
            } catch (FileVersionException | ParseException | java.io.IOException e) {
                failedPointTowardsPaths.add(pathName);
                DriverStation.reportWarning(
                        "Failed to preload point towards zones for path: " + pathName,
                        e.getStackTrace());
            }
        }
    }

    private void preloadPointTowardsZoneTriggers(PathPlannerPath path) {
        for (var zone : path.getPointTowardsZones()) {
            String zoneName = zone.name();
            if (!pointTowardsZoneTriggers.containsKey(zoneName)) {
                pointTowardsZoneTriggers.put(zoneName, new PointTowardsZoneTrigger(zoneName));
            }
        }
    }

    private PathPlannerPath getActivePreloadedPointTowardsPath(String currentPathName) {
        if (AutoBuilder.shouldFlip()) {
            return preloadedFlippedPointTowardsPaths.get(currentPathName);
        }
        return preloadedPointTowardsPaths.get(currentPathName);
    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        File autoDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        File[] autoFiles = autoDirectory.listFiles((dir, name) -> name.endsWith(".auto"));

        if (autoFiles == null || autoFiles.length == 0) {
            chooser.setDefaultOption("None", Commands.none());
            return chooser;
        }

        Arrays.sort(autoFiles, Comparator.comparing(File::getName, String.CASE_INSENSITIVE_ORDER));

        String defaultAutoName = Arrays.stream(autoFiles)
                .map(File::getName)
                .map(fileName -> fileName.substring(0, fileName.lastIndexOf('.')))
                .filter("Test"::equals)
                .findFirst()
                .orElseGet(() -> {
                    String firstFileName = autoFiles[0].getName();
                    return firstFileName.substring(0, firstFileName.lastIndexOf('.'));
                });

        for (File autoFile : autoFiles) {
            String fileName = autoFile.getName();
            String autoName = fileName.substring(0, fileName.lastIndexOf('.'));
            Command autoCommand = new PathPlannerAuto(autoName);

            if (defaultAutoName.equals(autoName)) {
                chooser.setDefaultOption(autoName, autoCommand);
            } else {
                chooser.addOption(autoName, autoCommand);
            }
        }

        chooser.addOption("None", Commands.none());
        return chooser;
    }

    private Command createDriveTuningCommand(double velocityX, double velocityY, double rotationalRate) {
        return drivetrain.applyRequest(() -> driveTuningRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate));
    }

    private SwerveRequest getDriverDriveRequest() {
        return drive.withVelocityX(-driversController.getLeftY() * MaxSpeed)
                .withVelocityY(-driversController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driversController.getRightX() * MaxAngularRate);
    }

    double shooterTuneSpeed = 0;

    private double getTuningShooterPower() {
        return shooterTuneSpeed;
    }

    private double getDesiredShooterIps() {
        if (pathPlannerAutoAssist.hasShotOutputsActive()) {
            return pathPlannerAutoAssist.getCommandedFlywheelIps();
        }
        return getTuningShooterPower();
    }

    double hoodTuneAngle = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    double activeIntakeAngleDegrees = DEFAULT_ACTIVE_INTAKE_ANGLE_DEGREES;


    private Angle getHoodTuningAngle() {
        return Degrees.of(hoodTuneAngle);
    }

    private Angle getDesiredHoodAngle() {
        if (pathPlannerAutoAssist.hasShotOutputsActive()) {
            return Degrees.of(pathPlannerAutoAssist.getCommandedHoodAngleDegrees());
        }
        return getHoodTuningAngle();
    }

    private Angle getDesiredHorizontalAimAngle() {
        if (pathPlannerAutoAssist.hasShotOutputsActive()) {
            return Degrees.of(pathPlannerAutoAssist.getCommandedTurretDeltaDegrees());
        }
        return Degrees.of(0.0);
    }

    private Angle getDesiredIntakeAngle() {
        if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
            return Degrees.of(80.0);
        }
        if (pathPlannerAutoAssist.isIntakeEnabled()) {
            return Degrees.of(PathPlannerAutoAssist.AUTO_INTAKE_DEPLOY_ANGLE_DEGREES);
        }
        if (isManualIntakeRequested()) {
            return Degrees.of(activeIntakeAngleDegrees);
        }
        return Degrees.of(5.0);
    }

    private double getDesiredIntakeDriveIps() {
        if (CommandScheduler.getInstance().isScheduled(dataCollectionCommand)) {
            return 0.0;
        }
        if (pathPlannerAutoAssist.isIntakeEnabled()) {
            return PathPlannerAutoAssist.AUTO_INTAKE_DRIVE_SPEED_IPS;
        }
        return isManualIntakeRequested() ? 300.0 : 0.0;
    }

    private boolean isManualIntakeRequested() {
        return engineersController.a().getAsBoolean()
                || driversController.x().getAsBoolean();
    }

    private void adjustHoodTuneAngle(double deltaDegrees) {
        hoodTuneAngle = Math.max(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                Math.min(ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES, hoodTuneAngle + deltaDegrees));
        // Debug tuning telemetry disabled to reduce NetworkTables traffic.
        // hoodTuneAnglePublisher.set(hoodTuneAngle);
    }

    private void adjustShooterTuneSpeed(double deltaIps) {
        shooterTuneSpeed += deltaIps;
        // Debug tuning telemetry disabled to reduce NetworkTables traffic.
        // shooterTuneSpeedPublisher.set(shooterTuneSpeed);
    }

    private void adjustActiveIntakeAngle(double deltaDegrees) {
        activeIntakeAngleDegrees = Math.max(
                intakePosition.getMinimumAngle().in(Degrees),
                Math.min(
                        intakePosition.getMaximumAngle().in(Degrees),
                        activeIntakeAngleDegrees + deltaDegrees));
    }

    public void publishTuningTelemetry() {
        // Debug tuning telemetry disabled to reduce NetworkTables traffic.
        // hoodTuneAnglePublisher.set(hoodTuneAngle);
        // shooterTuneSpeedPublisher.set(shooterTuneSpeed);
        // mainFlywheelMeasuredSpeedPublisher.set(shooter.getMainFlywheelSpeedIPS());
    }

    public void publishGameplayTelemetry() {
        GameplayDashboard.publishEngineersTarget(
                drivetrain.getState().Pose,
                getEngineersTarget());
    }

}
