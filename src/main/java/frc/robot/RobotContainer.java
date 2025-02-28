package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser ;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;
import java.util.Objects;


public class RobotContainer {
    // Controllers
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

//    private GenericHID operator2;
//    private final JoystickButton test = new JoystickButton(operator2, 1);
//    private final JoystickButton test2 = new JoystickButton(operator2, GenericHID.HIDType.kHIDGamepad.value);

//    private final BetterPoseEstimator poseEstimator = new BetterPoseEstimator();
//    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(poseEstimator);
//    private final Pickup pickup = new Pickup();
    private final Elevator elevator = new Elevator();
//    private final Climber climber = new Climber();
    private final Delivery delivery  = new Delivery();


    // Shuffleboard
    private final SendableChooser<Double> driveSpeed = new SendableChooser<>();
    private final SendableChooser<Double> turnSpeed = new SendableChooser<>();

    private final Field2d limelightField = new Field2d();
    private final Field2d odometryField = new Field2d();
    private final Field2d poseEstimatorField = new Field2d();
    private final Field2d anchorField = new Field2d();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("keystrokes");
    ManipulatorButtons manipulatorButtons = new ManipulatorButtons(table.getStringTopic("key"));

    private int i = 0;

    public RobotContainer() {
        ShuffleboardTab drivingTab = Shuffleboard.getTab("Driving");
        GenericEntry ShooterAngleEntry = drivingTab.add("Shooter angle", 0).withPosition(0, 0).withSize(2, 1).getEntry();
        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
//        AutoBuilder.configure(
//                swerveDrivetrain::getOdometryPose, // Robot pose supplier
//                swerveDrivetrain::setOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
//                swerveDrivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
////                (speeds, feedforwards) -> swerveDrivetrain.pathPlannerDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
//                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
//                ),config,
//                () -> {
//                    // Boolean supplier that controls when the path will be mirrored for the red
//                    // alliance
//                    // This will flip the path being followed to the red side of the field.
//                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//
//                    var alliance = DriverStation.getAlliance();
//                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
//                },
//                swerveDrivetrain // Reference to this subsystem to set requirements
//        );

       // Register Named Commands
        NamedCommands.registerCommand("extendWrist", new InstantCommand(this::doNothing));

 //       autonomousSelector = AutoBuilder.buildAutoChooser();
        // drivingTab.add( new HttpCamera("limelight",
        //                 NetworkTableInstance.getDefault().getEntry("limelight_Stream").getString("http://limelight.local:5800/stream.mjpg"),
        //                 HttpCamera.HttpCameraKind.kMJPGStreamer))
        //         .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        //         .withPosition(2,1)
        //         .withSize(8,4);

        setupShuffleboard(drivingTab);
//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());
        configureControllerBindings();
        // configureControllerBindingsTeddy();
    }

    public void doNothing() {

    }

    
    private void setupShuffleboard(ShuffleboardTab drivingTab) {


        driveSpeed.setDefaultOption("100%", 1.0);
        driveSpeed.addOption("80%", 0.8);
        driveSpeed.addOption("50%", 0.5);
        driveSpeed.addOption("25%", 0.25);
        driveSpeed.addOption("15%", 0.15);
        driveSpeed.addOption("10%", 0.1);


        turnSpeed.setDefaultOption("100%", 1.0);
        turnSpeed.setDefaultOption("80%", 0.8);
        turnSpeed.addOption("50%", 0.5);
        turnSpeed.addOption("25%", 0.25);
        turnSpeed.addOption("15%", 0.15);
        turnSpeed.addOption("10%", 0.1);


        //drivingTab.add("Autonomous", autonomousSelector).withPosition(2, 0).withSize(2, 1);
        drivingTab.add("Drive Speed", driveSpeed).withPosition(0, 1).withSize(2, 1);
        drivingTab.add("Turning Speed", turnSpeed).withPosition(2, 1).withSize(2, 1);


        //private final SendableChooser<Command> autonomousSelector;
        GenericEntry fieldOrientedBooleanBox = drivingTab.add("FieldOriented", true).withPosition(4, 1).withSize(2, 2).getEntry();

        SmartDashboard.putData("Limelight Position", limelightField);
        SmartDashboard.putData("Odometry Position", odometryField);
        SmartDashboard.putData("Pose Estimator", poseEstimatorField);
        SmartDashboard.putData("Anchor Position", anchorField);

        Shuffleboard.selectTab("Driving");
        Shuffleboard.update();
    }

    private void configureControllerBindings() {
//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());


        // manipulatorController.rightTrigger().whileTrue(Commands.repeatingSequence(new InstantCommand(() -> shooter.autoAngle(poseEstimator)), new WaitCommand(0.1)));
        
        
//        driveController.a().onTrue(
//                Commands.runOnce(swerveDrivetrain::resetGyro).andThen(
//                Commands.runOnce(swerveDrivetrain::resetOdometry)
//                )
//        );
//
//        driveController.rightBumper().onTrue(Commands.runOnce(swerveDrivetrain::toggleFieldOriented));
//
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

        manipulatorController.leftBumper().onTrue(Commands.runOnce(elevator::ManualAdjustIn));
        manipulatorController.rightBumper().onTrue(Commands.runOnce(elevator::manualAdjustOut));

        manipulatorController.a().onTrue(delivery.runConveyorOut());
        manipulatorController.a().onFalse(delivery.stopConveyorCmd());

        manipulatorController.x().onTrue(delivery.runConveyorIn());
        manipulatorController.x().onFalse(delivery.stopConveyorCmd());


        manipulatorController.b().onTrue(delivery.runDeliveryOut());
        manipulatorController.b().onFalse(delivery.stopDeliveryCmd());

        manipulatorController.y().onTrue(delivery.runDeliveryIn());
        manipulatorController.y().onFalse(delivery.stopDeliveryCmd());

        manipulatorController.start().onTrue(delivery.runConveyorOut());
        manipulatorController.start().onTrue(delivery.runDeliveryOut());
        manipulatorController.start().onFalse(delivery.stopDeliveryCmd());
        manipulatorController.start().onFalse(delivery.stopConveyorCmd());




//        manipulatorController.b().onTrue(Commands.runOnce(pickup::lowerPickup));
//        manipulatorController.b().onFalse(Commands.runOnce(pickup::raisePickup));
//
//        manipulatorController.a().onTrue(Commands.runOnce(delivery::runDeliveryOut));
//        manipulatorController.a().onFalse(Commands.runOnce(delivery::stopDelivery));
//
//        manipulatorController.x().onTrue(Commands.runOnce(delivery::runDeliveryIn));
//        manipulatorController.x().onFalse(Commands.runOnce(delivery::stopDelivery));


    }

//    public Command getSwerveDriveCommand() {
//        XboxController controller = driveController.getHID();
////        return new InstantCommand(() -> swerveDrivetrain.driveWithController(controller, driveSpeed, turnSpeed), swerveDrivetrain);
//    }

    public Command getAutonomousCommand() {
        //return autonomousSelector.getSelected();
        return null;
    }

    public void sleep(double millis) {
      try {
            Thread.sleep(100);
        } catch (Exception e) {
            return;
        }
    }

    public void robotInit() {
//        swerveDrivetrain.stop();
//        swerveDrivetrain.resetGyro();

    }

    public void teleopInit() {


        // Reset the braking state in case autonomous exited uncleanly
        System.out.println("=======================================================");
        System.out.println("==================Starting teleop======================");
        System.out.println("=======================================================");

//        swerveDrivetrain.init();
//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());

//        climber.recalibrateClimber();

    }


    public void autonomousInit() {
        System.out.println("=======================================================");
        System.out.println("================Starting Autonomous====================");
        System.out.println("=======================================================");
//        swerveDrivetrain.setDefaultCommand(getSwerveDriveCommand());


    }


    public void periodic() {
        String key = manipulatorButtons.getKey();
        if (!Objects.equals(key, "")) {
            if (Objects.equals(key, "a")) {
                System.out.println("Transitioning to state 0");
                elevator.transitionToState(0);
            }else if (Objects.equals(key, "b")) {
                System.out.println("Transitioning to state 1");
                elevator.transitionToState(1);
            } else if (Objects.equals(key, "c")) {
                System.out.println("Transitioning to state 2");
                elevator.transitionToState(2);
            }else if (Objects.equals(key, "d")) {
                System.out.println("Transitioning to state 3");
                elevator.transitionToState(3);
            }else if (Objects.equals(key, "e")) {
                System.out.println("Transitioning to state 4");
                elevator.transitionToState(4);
            }
            System.out.println(key);
        }

        i++;
        if (i % 20 == 0) {
            elevator.debug();

        }
//        odometryField.setRobotPose(swerveDrivetrain.getOdometryPose());
//        poseEstimatorField.setRobotPose(poseEstimator.getCurrentPose());
//        anchorField.setRobotPose(poseEstimator.getAnchor());
//        fieldOrientedBooleanBox.setBoolean(swerveDrivetrain.fieldOriented);
    }

    public void teleopPeriodic() {

    }

    public void enable() {

    }

    public void disable() {

//        swerveDrivetrain.stop();
    }
}
