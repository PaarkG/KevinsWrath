package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX motor;

    public IntakeSubsystem() {
        this.motor = new TalonFX(IntakeConstants.kMotorID);
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
        return setPercentage(IntakeConstants.kIntakePercent);
    }

    public Command setOuttakeState() {
        return setPercentage(IntakeConstants.kOuttakePercent);
    }

    private static final class IntakeConstants {
        private static final int kMotorID = -1;

        private static final double kIntakePercent = .5;
        private static final double kOuttakePercent = -.5;
    }
}
