package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Climber.*;


public class Climber extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(Constants.Climber.Ports.CLIMBER_MOTOR);
    private final PIDController climberPID = new PIDController(KP, KI, KD);
    public double pidOutput = 0;

    public Climber() {
        climberPID.reset();
    }

    /**
     * Updates the field relative position of the robot.
     * Called automatically by command scheduler
     */
    @Override
    public void periodic() {
        pidOutput = climberPID.calculate(climberMotor.getPosition().getValueAsDouble());
        //climberMotor.set(pidOutput);
    }


    public void climb() {
        climberPID.setSetpoint(CLIMBING_SETPOINT);
    }
    public void dock() {
        climberPID.setSetpoint(DOCKING_SETPOINT);
    }
}
