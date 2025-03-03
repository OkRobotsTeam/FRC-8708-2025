package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser ;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import frc.robot.commands.DoNothingFor2Sec;
import frc.robot.subsystems.*;

import static frc.robot.Constants.Delivery.CONVEYOR_OUT_SPEED;
import static frc.robot.Constants.Pickup.RAISED_SETPOINT;
import static java.lang.Math.abs;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    private final BetterPoseEstimator poseEstimator = new BetterPoseEstimator();
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(poseEstimator);
    //    private final Pickup pickup = new Pickup();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final Delivery delivery = new Delivery();

    // Shuffleboard
    private final SendableChooser<Double> driveSpeed = new SendableChooser<>();
    private final SendableChooser<Double> turnSpeed = new SendableChooser<>();
    private final SendableChooser<Command> autonomousSelector;
    ShuffleboardTab drivingTab = Shuffleboard.getTab("Driving");
    GenericEntry fieldOrientedBooleanBox = drivingTab.add("FieldOriented", true).withPosition(4, 1).withSize(2, 2).getEntry();


    private final Field2d limelightField = new Field2d();
    private final Field2d odometryField = new Field2d();
    private final Field2d poseEstimatorField = new Field2d();
    private final Field2d anchorField = new Field2d();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("keystrokes");
    ManipulatorButtons manipulatorButtons = new ManipulatorButtons(table.getStringTopic("key"));

    private int i = 0;

    public RobotContainer() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        AutoBuilder.configure(
                swerveDrivetrain::getOdometryPose, // Robot pose supplier
                swerveDrivetrain::setOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> swerveDrivetrain.pathPlannerDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ), config, () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                swerveDrivetrain // Reference to this subsystem to set requirements
        );

        // Register Named Commands
        NamedCommands.registerCommand("extendWrist", new InstantCommand(this::doNothing));
        NamedCommands.registerCommand("raiseElevatorTo4", new InstantCommand(() -> elevator.transitionToState(4)));
        NamedCommands.registerCommand("raiseElevatorTo3", new InstantCommand(() -> elevator.transitionToState(3)));
        NamedCommands.registerCommand("raiseElevatorTo2", new InstantCommand(() -> elevator.transitionToState(2)));
        NamedCommands.registerCommand("raiseElevatorTo1", new InstantCommand(() -> elevator.transitionToState(1)));
        NamedCommands.registerCommand("lowerElevator", new InstantCommand(() -> elevator.transitionToState(0)));
        NamedCommands.registerCommand("deliver", delivery.runConveyorOut().andThen(delivery.runDeliveryOut()));
        NamedCommands.registerCommand("stopDelivery", delivery.stopDeliveryCmd().andThen(delivery.stopConveyorCmd()));
//        NamedCommands.registerCommand("doNothingFor2Seconds", Commands.run(DoNothingFor2Sec(swerveDrivetrain)));


        autonomousSelector = AutoBuilder.buildAutoChooser();
        // drivingTab.add( new HttpCamera("limelight",
        //                 NetworkTableInstance.getDefault().getEntry("limelight_Stream").getString("http://limelight.local:5800/stream.mjpg"),
        //                 HttpCamera.HttpCameraKind.kMJPGStreamer))
        //         .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        //         .withPosition(2,1)
        //         .withSize(8,4);

        setupShuffleboard(drivingTab);
//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());
        configureControllerBindings();
    }

    public void doNothing() {

    }


    private void setupShuffleboard(ShuffleboardTab drivingTab) {


        driveSpeed.setDefaultOption("100%", 1.0);
        driveSpeed.addOption("80%", 0.8);
        driveSpeed.addOption("75%", 0.75);
        driveSpeed.addOption("50%", 0.5);
        driveSpeed.addOption("25%", 0.25);
        driveSpeed.addOption("15%", 0.15);
        driveSpeed.addOption("10%", 0.1);


        turnSpeed.addOption("100%", 1.0);
        turnSpeed.addOption("80%", 0.8);
        turnSpeed.setDefaultOption("75%", 0.75);
        turnSpeed.addOption("50%", 0.5);
        turnSpeed.addOption("25%", 0.25);
        turnSpeed.addOption("15%", 0.15);
        turnSpeed.addOption("10%", 0.1);


        drivingTab.add("Autonomous", autonomousSelector).withPosition(2, 0).withSize(2, 1);
        drivingTab.add("Drive Speed", driveSpeed).withPosition(0, 1).withSize(2, 1);
        drivingTab.add("Turning Speed", turnSpeed).withPosition(2, 1).withSize(2, 1);

        drivingTab.add(new HttpCamera("Left Camera",
                        "http://10.87.8.201:1183/stream.mjpg",
                        HttpCamera.HttpCameraKind.kMJPGStreamer))
                .withPosition(6, 1)
                .withSize(4, 2);

        SmartDashboard.putData("Limelight Position", limelightField);
        SmartDashboard.putData("Odometry Position", odometryField);
        SmartDashboard.putData("Pose Estimator", poseEstimatorField);
        SmartDashboard.putData("Anchor Position", anchorField);

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }

    private void configureControllerBindings() {
        driveController.a().onTrue(
                Commands.runOnce(swerveDrivetrain::resetGyro).andThen(
                        Commands.runOnce(swerveDrivetrain::resetOdometry)
                )
        );

//        driveController.rightBumper().onTrue(Commands.runOnce(swerveDrivetrain::toggleFieldOriented));

//        manipulatorController.rightTrigger().onTrue(Commands.runOnce(pickup::runIntakeIn));
//        manipulatorController.leftTrigger().onTrue(Commands.runOnce(pickup::runIntakeOut));
//        manipulatorController.rightTrigger().onFalse(Commands.runOnce(pickup::stopIntake));
//        manipulatorController.leftTrigger().onFalse(Commands.runOnce(pickup::stopIntake));
//
//        manipulatorController.rightTrigger().and(() -> (pickup.rotationPID.getSetpoint() != RAISED_SETPOINT)).onTrue(
//                Commands.runOnce(pickup::runIntakeIn).andThen(Commands.runOnce(delivery::runDeliveryOut)));
//
//        manipulatorController.leftTrigger().onTrue(Commands.runOnce(pickup::runIntakeOut));
//        manipulatorController.rightTrigger().onFalse(Commands.runOnce(pickup::stopIntake));
//        manipulatorController.leftTrigger().onFalse(Commands.runOnce(pickup::stopIntake));

        manipulatorController.povUp().onTrue(Commands.runOnce(elevator::nextState));
        manipulatorController.povDown().onTrue(Commands.runOnce(elevator::previousState));

        manipulatorController.leftBumper().and(manipulatorController.rightBumper().negate()).onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.rightBumper().and(manipulatorController.leftBumper().negate()).onTrue(Commands.runOnce(() -> climber.setSpeed(-1.0)));
        manipulatorController.leftBumper().and(manipulatorController.rightBumper()).onTrue(Commands.runOnce(() -> climber.setSpeed(1.0)));
        manipulatorController.leftBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));
        manipulatorController.rightBumper().onFalse(Commands.runOnce(() -> climber.setSpeed(0.0)));


//        manipulatorController.leftBumper().onTrue(Commands.runOnce(elevator::ManualAdjustIn));
//        manipulatorController.rightBumper().onTrue(Commands.runOnce(elevator::manualAdjustOut));


//        manipulatorController.b().onTrue(Commands.runOnce(() -> delivery.setConveyorMotor(-0.1)));

        manipulatorController.b().onTrue(delivery.runConveyorOut());
        manipulatorController.b().onFalse(delivery.stopConveyorCmd());

        manipulatorController.a().onTrue(delivery.runConveyorIn());
        manipulatorController.a().onFalse(delivery.stopConveyorCmd());

        manipulatorController.x().onTrue(Commands.runOnce(elevator::toggleHigh));
//`
//        manipulatorController.b().onTrue(delivery.runDeliveryOut());
//        manipulatorController.b().onFalse(delivery.stopDeliveryCmd());
//
//        manipulatorController.y().onTrue(delivery.runDeliveryIn());
//        manipulatorController.y().onFalse(delivery.stopDeliveryCmd());

        manipulatorController.rightTrigger().onTrue(Commands.runOnce(() -> delivery.setConveyorMotor(CONVEYOR_OUT_SPEED)).andThen(() -> System.out.println("POSITION: " + swerveDrivetrain.getOdometryPose() + " ELEVATOR STATE: " + elevator.getElevatorPosition())));
        manipulatorController.rightTrigger().onTrue(delivery.runDeliveryOut());
        manipulatorController.rightTrigger().onFalse(delivery.stopDeliveryCmd());
        manipulatorController.rightTrigger().onFalse(delivery.stopConveyorCmd());

        manipulatorController.leftTrigger().onTrue(delivery.runConveyorIn());
        manipulatorController.leftTrigger().onTrue(delivery.runDeliveryIn());
        manipulatorController.leftTrigger().onFalse(delivery.stopDeliveryCmd());
        manipulatorController.leftTrigger().onFalse(delivery.stopConveyorCmd());


//        manipulatorController.b().onTrue(Commands.runOnce(pickup::lowerPickup));
//        manipulatorController.b().onFalse(Commands.runOnce(pickup::raisePickup));
//
//        manipulatorController.a().onTrue(Commands.runOnce(delivery::runDeliveryOut));
//        manipulatorController.a().onFalse(Commands.runOnce(delivery::stopDelivery));
//
//        manipulatorController.x().onTrue(Commands.runOnce(delivery::runDeliveryIn));
//        manipulatorController.x().onFalse(Commands.runOnce(delivery::stopDelivery));
    }

    public Command getSwerveDriveCommand() {
        XboxController controller = driveController.getHID();
        return new InstantCommand(() -> swerveDrivetrain.driveWithController(controller, driveSpeed, turnSpeed, elevator.getElevatorPosition()), swerveDrivetrain);
    }

    public Command getAutonomousCommand() {
        return autonomousSelector.getSelected();
    }


    public void robotInit() {
        swerveDrivetrain.stop();
        swerveDrivetrain.resetGyro();
    }

    public void teleopInit() {


        // Reset the braking state in case autonomous exited uncleanly
        System.out.println("=======================================================");
        System.out.println("==================Starting teleop======================");
        System.out.println("=======================================================");

//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

//        climber.recalibrateClimber();

    }


    public void autonomousInit() {
        System.out.println("=======================================================");
        System.out.println("================Starting Autonomous====================");
        System.out.println("=======================================================");
//        swerveDrivetrain.setDefaultCommand(new InstantCommand(swerveDrivetrain::stop, swerveDrivetrain));
    }


    public void periodic() {
        odometryField.setRobotPose(swerveDrivetrain.getOdometryPose());
        poseEstimatorField.setRobotPose(poseEstimator.getCurrentPose());
        anchorField.setRobotPose(poseEstimator.getAnchor());
        fieldOrientedBooleanBox.setBoolean(swerveDrivetrain.fieldOriented);
    }

    public void autonomousPeriodic() {
        elevator.teleopPeriodic();
    }

    public void teleopPeriodic() {
        getSwerveDriveCommand().schedule();
        elevator.teleopPeriodic();
        double manualAdjustAmount = manipulatorController.getLeftY();
        if (abs(manualAdjustAmount) < 0.2) {
            manualAdjustAmount = 0;
        }

        elevator.manualAdjust(manualAdjustAmount * 6);

//        manipulatorButtons.checkButtons(elevator);
        i++;
        if (i % 20 == 0) {
            elevator.debug();
        }
    }

    public void enable() {
        elevator.transitionToState(0);
        swerveDrivetrain.init();
    }

    public void disable() {
        elevator.transitionToState(0);
//        swerveDrivetrain.stop();
    }

    public void testInit() {
        System.out.println("=======================================================");
        System.out.println("==================Starting test========================");
        System.out.println("=======================================================");
        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());
    }

    private void configureTestControllerBindings() {
//        driveController.a().onTrue(swerveDrivetrain.moduleTest(0));
//        driveController.b().onTrue(swerveDrivetrain.moduleTest(1));
//        driveController.x().onTrue(swerveDrivetrain.moduleTest(2));
//        driveController.y().onTrue(swerveDrivetrain.moduleTest(3));



        driveController.a().onFalse(Commands.runOnce(() -> { swerveDrivetrain.drive(0,0,0,false);} ));
        driveController.b().onFalse(Commands.runOnce(() -> { swerveDrivetrain.drive(0,0,0,false);} ));
        driveController.x().onFalse(Commands.runOnce(() -> { swerveDrivetrain.drive(0,0,0,false);} ));
        driveController.y().onFalse(Commands.runOnce(() -> { swerveDrivetrain.drive(0,0,0,false);} ));

    }
}