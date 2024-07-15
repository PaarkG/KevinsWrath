package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    private TalonFX motor;

    public IndexSubsystem() {
        motor = new TalonFX(IndexConstants.kMotorID);
    }

    private Command setVoltage(double voltage) {
        return this.runOnce(() -> motor.setVoltage(voltage));
    }

    private Command setPercentage(double percentage) {
        return this.runOnce(() -> motor.set(percentage));
    }

    public Command setIdleState() {
        return setVoltage(0);
    }

    public Command setIntakeState() {
        return setPercentage(IndexConstants.kIntakePercent);
    }

    public Command setOuttakeState() {
        return setPercentage(IndexConstants.kOuttakePercent);
    }

    public Command setFeedState() {
        return setPercentage(IndexConstants.kFeedPercent);
    }

    public Command setAmpState() {
        return setPercentage(IndexConstants.kAmpPercent);
    }

    @Override
    public Command getDefaultCommand() {
        return setIdleState();
    }

    private static final class IndexConstants {
        private static final int kMotorID = -1;

        private static final double kIntakePercent = .3;
        private static final double kOuttakePercent = -.3;
        private static final double kFeedPercent = .5;
        private static final double kAmpPercent = -.5;
    }
}
