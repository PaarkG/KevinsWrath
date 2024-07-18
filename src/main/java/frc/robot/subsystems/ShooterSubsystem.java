package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private Slot0Configs pidConfigs;
    private double leftTarget;
    private double rightTarget;

    public ShooterSubsystem() {
        leftMotor = new TalonFX(ShooterConstants.kLeftID);
        rightMotor = new TalonFX(ShooterConstants.kRightID);
        pidConfigs = new Slot0Configs();
        pidConfigs.kP = ShooterConstants.kP;
        pidConfigs.kD = ShooterConstants.kD;
        leftMotor.getConfigurator().apply(pidConfigs);
        rightMotor.getConfigurator().apply(pidConfigs);
        leftTarget = 0.0;
        rightTarget = 0.0;
    }

    /** ROTATIONS PER SECOND, NOT MINUTE. DIVIDE RPM BY 60 */
    private Command setVelocity(double leftVelocity, double rightVelocity) {
        return this.run(() -> {
            leftTarget = leftVelocity;
            rightTarget = rightVelocity;
            leftMotor.setControl(new VelocityVoltage(leftVelocity));
            rightMotor.setControl(new VelocityVoltage(rightVelocity));
        });
    }

    public Command setIdleState() {
        return setVelocity(0, 0);
    }

    public Command setDefaultShootVelocity() {
        return setVelocity(ShooterConstants.kDefaultShootLeft, ShooterConstants.kDefaultShootRight);
    }

    public Command windUpDefault() {
        return setDefaultShootVelocity().repeatedly().until(atSpeed());
    }

    public Command windUp(double leftVelocity, double rightVelocity) {
        return setVelocity(leftVelocity, rightVelocity).repeatedly().until(atSpeed());
    }

    public BooleanSupplier atSpeed() {
        return () -> (rightTarget - rightMotor.getVelocity().getValueAsDouble()) < ShooterConstants.kTolerance 
            && (leftTarget - leftMotor.getVelocity().getValueAsDouble()) < ShooterConstants.kTolerance;
    }

    @Override
    public Command getDefaultCommand() {
        return setIdleState();
    }

    private static final class ShooterConstants {
        private static final int kLeftID = 1;
        private static final int kRightID = 2;
        private static final double kP = .01;
        private static final double kD = .0001;

        private static final double kDefaultShootLeft = 8000.0 / 60.0;
        private static final double kDefaultShootRight = 4000.0 / 60.0;

        private static final double kTolerance = 50.0;
    }
}
