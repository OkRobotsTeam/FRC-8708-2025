package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class DoNothingFor2Sec extends Command {

    double startTime = 0;
    SwerveDrivetrain drivetrain;

    public DoNothingFor2Sec(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        startTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() / 1000.0) - startTime > 2.0;
    }
}
