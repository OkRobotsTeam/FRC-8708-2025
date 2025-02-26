package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Delivery.*;


public class Delivery extends SubsystemBase {
    private final SparkMax deliveryMotor = new SparkMax(Ports.DELIVERY_MOTOR, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxConfig deliveryMotorConfig = new SparkMaxConfig();

    public Delivery() {
        deliveryMotorConfig.inverted(DELIVERY_MOTOR_INVERTED);
        deliveryMotorConfig.smartCurrentLimit(CURRENT_LIMIT_STALLED, CURRENT_LIMIT_FREE);
        deliveryMotor.configure(deliveryMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        stopDelivery();
    }

    public void runDeliveryIn() {
        setDeliveryMotor(DELIVERY_IN_SPEED);
    }

    public void runDeliveryOut() {
        setDeliveryMotor(DELIVERY_OUT_SPEED);
    }

    public void stopDelivery() {
        setDeliveryMotor(0.0);
    }

    public void setDeliveryMotor(double power) {
        deliveryMotor.set(power);
    }
}
