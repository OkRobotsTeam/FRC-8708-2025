package frc.robot.subsystems;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorButtons extends SubsystemBase {
    private final StringSubscriber keySubscriber;
    private double lastChange = 0;

    public ManipulatorButtons(StringTopic keyTopic) {
        keySubscriber = keyTopic.subscribe("");
    }

    public String getKey() {
        double thisChange = keySubscriber.getLastChange();
        if (thisChange > lastChange) {
            lastChange = thisChange;
            return keySubscriber.get();
        } else {
            return "";
        }

    }

}

