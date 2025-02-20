package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.Elevator.*;


public class Elevator extends SubsystemBase {
    private final PWMSparkMax motor1 = new PWMSparkMax(Ports.MOTOR_1);
    private final PWMSparkMax motor2 = new PWMSparkMax(Ports.MOTOR_2);
    Encoder encoder = new Encoder(Ports.ENCODER_CHANNEL_A, Ports.ENCODER_CHANNEL_B, ENCODER_REVERSED, ENCODER_ENCODING_TYPE);
    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(KP, KI, KD, ELEVATOR_TRAPEZOID_PROFILE);
    private double currentState = States.STATE1;
    private double manualAdjustAmount = 0.0;

    public Elevator() {
        stop();
        encoder.reset();
        // Configures the encoder to return a distance of 1 for every 8192 pulses (one revolution of the REV Through-bore)
        // Also changes the units of getRate
        encoder.setDistancePerPulse(1.0/8192.0);
        elevatorPID.reset(getElevatorPosition());
        transitionToState(currentState);
    }

    /**
     * Updates the field relative position of the robot.
     * Called automatically by command scheduler
     */
    @Override
    public void periodic() {
        double pidOutput = elevatorPID.calculate(getElevatorPosition());
        setMotors(pidOutput);
    }

    public double getElevatorPosition() {
        return encoder.getDistance();
    }

    public void transitionToState(double state) {
        elevatorPID.setGoal(state + manualAdjustAmount);
    }

    public void stop() {
        setMotors(0.0);
    }

    public void setMotors(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void manualAdjustOut() {
        manualAdjustAmount += MANUAL_ADJUST_STEP;
        transitionToState(currentState);
    }

    public void setManualAdjustIn() {
        manualAdjustAmount -= MANUAL_ADJUST_STEP;
        transitionToState(currentState);
    }
}
