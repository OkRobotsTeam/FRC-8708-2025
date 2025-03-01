package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Debug;


import java.sql.Time;

import static frc.robot.Constants.Elevator.*;


public class Elevator extends SubsystemBase {
    private final PWMSparkMax motor1 = new PWMSparkMax(Ports.MOTOR_1);
    private final PWMSparkMax motor2 = new PWMSparkMax(Ports.MOTOR_2);
    Encoder encoder = new Encoder(Ports.ENCODER_CHANNEL_A, Ports.ENCODER_CHANNEL_B, ENCODER_REVERSED, ENCODER_ENCODING_TYPE);

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(KP, KI, KD, ELEVATOR_TRAPEZOID_PROFILE);
    private int currentState = 0;
    private double manualAdjustAmount = 0.0;

    public Elevator() {
        stop();
        encoder.reset();
        // Configures the encoder to return a distance of 1 for every 8192 pulses (one revolution of the REV Through-bore)
        // Also changes the units of getRate
        encoder.setDistancePerPulse(20.0/2462.0);
        elevatorPID.reset(getElevatorPosition());
        transitionToState(currentState);
    }

    public void teleopPeriodic() {
        double pidOutput = elevatorPID.calculate(getElevatorPosition());
        setMotors(pidOutput + KG);
    }

    public double getElevatorPosition() {
        return encoder.getDistance();
    }

    public void transitionToState(int state) {
        try {
            elevatorPID.setGoal(States[state] + manualAdjustAmount);
            currentState = state;
        } catch (Exception e) {
            System.out.println("Error setting elevator to state: " + e);
        }
    }

    public void stop() {
        setMotors(0.0);
    }

    public void nextState() {
        if (currentState < States.length - 1) {
            transitionToState(currentState + 1);
        }
    }

    public void previousState() {
        if (currentState > 0) {
            transitionToState(currentState - 1);
        }
    }

    public void setMotors(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void manualAdjust(double amount) {
        manualAdjustAmount = manualAdjustAmount + amount * 0.02;
    }

    public void manualAdjustOut() {
        manualAdjustAmount += MANUAL_ADJUST_STEP;
        transitionToState(currentState);
    }

    public void ManualAdjustIn() {
        manualAdjustAmount -= MANUAL_ADJUST_STEP;
        transitionToState(currentState);
    }

    public void debug() {
        Debug.println("Encoder Distance: ", encoder.getDistance(), "  P Error: ", elevatorPID.getPositionError(), "  I Error: ", elevatorPID.getAccumulatedError(), "  Motor Power", motor1.get());
    }
}
