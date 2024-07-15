package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private TalonFX motor;
    private Slot0Configs pidConfigs;
    private double target;

    public PivotSubsystem() {
        motor = new TalonFX(PivotConstants.kMotorID);
        pidConfigs.kP = PivotConstants.kP;
        pidConfigs.kD = PivotConstants.kD;
        motor.getConfigurator().apply(pidConfigs);
        target = 0.0;
    }

    public Command setIntakeState() {
        target = 0.0;
        return run(() -> motor.setControl(new PositionVoltage(target))).until(atSetpoint());
    }

    public Command setAmpState() {
        target = 110.0 * PivotConstants.kPositionFactor;
        return run(() -> motor.setControl(new PositionVoltage(target))).until(atSetpoint());
    }

    public Command setSubwooferState() {
        target = 45.0 * PivotConstants.kPositionFactor;
        return run(() -> motor.setControl(new PositionVoltage(target))).until(atSetpoint());
    }

    public Command setPodiumState() {
        target = 25.0 * PivotConstants.kPositionFactor;
        return run(() -> motor.setControl(new PositionVoltage(target))).until(atSetpoint());
    }

    public BooleanSupplier atSetpoint() {
        return () -> (motor.getPosition().getValueAsDouble() - target) < PivotConstants.kTolerance;
    }

    @Override
    public Command getDefaultCommand() {
        return setIntakeState();
    }

    private static final class PivotConstants {
        private static final int kMotorID = -1;

        private static final double kP = 0.001;
        private static final double kD = 0.00001;

        private static final double kPositionFactor = 1.0 / 107.0;
        private static final double kTolerance = 1.0;
    }
}
