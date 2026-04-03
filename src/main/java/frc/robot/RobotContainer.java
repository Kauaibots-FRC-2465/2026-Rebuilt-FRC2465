
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;

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
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Commands.ScoreInHub;
import frc.robot.Commands.SnowblowToAlliance;
import frc.robot.Commands.SnowblowToAllianceWithOperatorAim;
import frc.robot.Commands.Rebound;
import frc.robot.Commands.HoodRestPositionCharacterization;
import frc.robot.Commands.HoodTimingCharacterization;
import frc.robot.Commands.ShooterConstants;
import frc.robot.Commands.TestShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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

// For tuning see https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
// Clockwise positive on Left flywheel
// Main flywheels are commanded independently; left is inverted relative to right
// kp=6
// ks=2.2
// kv=0.01

public class RobotContainer implements Subsystem {
    private final static float REMEMBER_TO_SET_CURRENTLY_0 = 0f;
    private final static float REMEMBER_TO_TUNE_CURRENTLY_1 = 1f;
    
    private final static int INCHES = 1;
    private static final double HOOD_TUNE_ANGLE_STEP_DEGREES = 5.0;
    private static final double SHOOTER_TUNE_SPEED_STEP_IPS = 40;
    private static final double MAIN_FLYWHEEL_VELOCITY_SAMPLE_WINDOW_SECONDS = 0.010;

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
    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
    private final DoublePublisher hoodTuneAnglePublisher = tuningTable.getDoubleTopic("HoodTuneAngleDegrees").publish();
    private final DoublePublisher shooterTuneSpeedPublisher = tuningTable.getDoubleTopic("ShooterTuneSpeedIps").publish();
    private final DoublePublisher mainFlywheelMeasuredSpeedPublisher =
            tuningTable.getDoubleTopic("MainFlywheelMeasuredSpeedIps").publish();

    private final CommandXboxController driversController = new CommandXboxController(0);
    private final CommandXboxController engineersController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
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

    public final SwerveDrivetrainSubsystem encapsulatedDrivetrain;

    public final SparkAnglePositionSubsystem horizontalAim;
    public final SparkFollowerSubsystem sparkFollowerSubsystem = new SparkFollowerSubsystem();

    public final SparkAnglePositionSubsystem verticalAim;
    public final IntakePositionSubsystem intakePosition;

    public final ShooterSubsystem shooter;
    public final KrakenFlywheelSubsystem intakedrive;
    private final TestShootingCommand testShootingCommand;
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
            Degrees.of(-22),
            Degrees.of(22),
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
            .2, //0.06,
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
            Degrees.of(110.0));


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
            30),
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
          20                 // | Peak current
        );
        testShootingCommand = new TestShootingCommand(
            poseEstimatorSubsystem,
            horizontalAim,
            verticalAim,
            shooter);
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
        shooter.setDefaultCommand(shooter.cmdSetCoupledIPS(this::getTuningShooterPower));

        intakePosition.setDefaultCommand(
            intakePosition.cmdSetAngle(() -> Degrees.of(engineersController.a().getAsBoolean() ? 109 : 5.0)));
        intakedrive.setDefaultCommand(intakedrive.cmdSetIPS(()->engineersController.a().getAsBoolean() ? 300.0 : 0.0)); //600 max

        //hood.setDefaultCommand(hood.cmdSetScaledAngle(engineersController::getLeftTriggerAxis));
        verticalAim.setDefaultCommand(verticalAim.cmdSetAngle(this::getHoodTuningAngle));
        engineersController.povUp().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(testShootingCommand)) {
                testShootingCommand.trimHoodUp();
            } else {
                adjustHoodTuneAngle(-HOOD_TUNE_ANGLE_STEP_DEGREES);
            }
        }));
        engineersController.povDown().onTrue(Commands.runOnce(() -> {
            if (CommandScheduler.getInstance().isScheduled(testShootingCommand)) {
                testShootingCommand.trimHoodDown();
            } else {
                adjustHoodTuneAngle(HOOD_TUNE_ANGLE_STEP_DEGREES);
            }
        }));
        engineersController.povRight().onTrue(Commands.runOnce(() -> adjustShooterTuneSpeed(SHOOTER_TUNE_SPEED_STEP_IPS)));
        engineersController.povLeft().onTrue(Commands.runOnce(() -> adjustShooterTuneSpeed(-SHOOTER_TUNE_SPEED_STEP_IPS)));
        engineersController.y().toggleOnTrue(testShootingCommand);
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
        //engineersController.leftBumper().onTrue(
        //    new HoodTimingCharacterization(verticalAim)
        //);
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

        driversController.x().onTrue(showLights);
        driversController.x().onFalse(showStatic);
        driversController.rightTrigger().whileTrue(
            new SnowblowToAllianceWithOperatorAim(
                drivetrain,
                poseEstimatorSubsystem,
                horizontalAim,
                verticalAim,
                shooter,
                this::getDriverDriveRequest,
                engineersController::getRightX)
        );
        driversController.leftTrigger().whileTrue(
            new ScoreInHub(
                drivetrain,
                poseEstimatorSubsystem,
                horizontalAim,
                verticalAim,
                shooter,
                this::getDriverDriveRequest)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driversController.back().and(driversController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driversController.back().and(driversController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driversController.start().and(driversController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driversController.start().and(driversController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

       //*  Seed the driver-perspective offset from the fused heading on left bumper press.
        driversController.leftBumper().onTrue(
            Commands.runOnce(() -> {
                Pose2d fusedPose = poseEstimatorSubsystem.getFusedPoseSupplier().get();
                if (fusedPose != null) {
                    drivetrain.seedDriverPerspectiveToHeading(fusedPose.getRotation());
                }
            })
        );

        horizontalAim.setDefaultCommand(
            horizontalAim.cmdSetScaledAngle(() -> (1.0 - engineersController.getLeftX()) / 2.0));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //NamedCommands.registerCommand("spinIntake", new SequentialCommandGroup(shooter.cmdSetCoupledIPS(() -> 1000)));
        //NamedCommands.registerCommand("deployIntake", exampleSubsystem.exampleCommand());
        //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

        return new PathPlannerAuto("Plow");
    }

    private void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }

    private double getShooterPower() {
        final double minPower = 0.4;
        double shooterPower = engineersController.getRightTriggerAxis();
        return (shooterPower < 0.005) ? 0.0 : (shooterPower + minPower) / (1.0 + minPower);
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

    double hoodTuneAngle = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;


    private Angle getHoodTuningAngle() {
        return Degrees.of(hoodTuneAngle);
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

    public void publishTuningTelemetry() {
        // Debug tuning telemetry disabled to reduce NetworkTables traffic.
        // hoodTuneAnglePublisher.set(hoodTuneAngle);
        // shooterTuneSpeedPublisher.set(shooterTuneSpeed);
        // mainFlywheelMeasuredSpeedPublisher.set(shooter.getMainFlywheelSpeedIPS());
    }

}
