package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Pickup.*;


public class Pickup extends SubsystemBase {
    private final SparkMax intakeMotor1 = new SparkMax(Ports.INTAKE_MOTOR_1, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax intakeMotor2 = new SparkMax(Ports.INTAKE_MOTOR_2, SparkLowLevel.MotorType.kBrushless);
    private final TalonFX rotationMotor = new TalonFX(Ports.ROTATION_MOTOR);
    private final PIDController rotationPID = new PIDController(KP, KI, KD);
    private SparkMaxConfig intakeMotor1Config = new SparkMaxConfig();
    private SparkMaxConfig intakeMotor2Config = new SparkMaxConfig();


    /**
     * Updates the field relative position of the robot.
     * Called automatically by command scheduler
     */
    @Override
    public void periodic() {
        double pidOutput = rotationPID.calculate(rotationMotor.getPosition().getValueAsDouble());
        rotationMotor.set(pidOutput);
    }
    
    public Pickup() {
        rotationPID.reset();
        intakeMotor1Config.inverted(MOTOR_1_INVERTED);
        intakeMotor2Config.inverted(MOTOR_1_INVERTED);
        intakeMotor1Config.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        intakeMotor2Config.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        intakeMotor1.configure(intakeMotor1Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeMotor2Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        stopIntake();
    }

    public void lowerPickup() {
        rotationPID.setSetpoint(LOWERED_SETPOINT);
    }

    public void raisePickup() {
        rotationPID.setSetpoint(RAISED_SETPOINT);
    }

    public void runIntakeIn() {
        setIntakeMotors(INTAKE_IN_SPEED);
    }

    public void runIntakeOut() {
        setIntakeMotors(INTAKE_OUT_SPEED);
    }

    public void stopIntake() {
        setIntakeMotors(0.0);
    }

    public void setIntakeMotors(double power) {
        intakeMotor1.set(power);
        intakeMotor2.set(power);
    }
}
