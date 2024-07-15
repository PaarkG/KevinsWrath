package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Conversions;

public class SwerveModule {
    private TalonFX driveMotor, angleMotor;
    private CANcoder angleEncoder;

    private Rotation2d angleOffset;

    private VelocityVoltage driveVelocity;
    private PositionVoltage positionVoltage;
    
    public SwerveModule(int driveID, int steerID, int encoderID, Rotation2d angleOffset) {
        driveMotor = new TalonFX(driveID);
        angleMotor = new TalonFX(steerID);
        angleEncoder = new CANcoder(encoderID);
        this.angleOffset = angleOffset;

        driveVelocity = new VelocityVoltage(0);
        positionVoltage = new PositionVoltage(0);
    }

    public void setModuleState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentState().angle);
        angleMotor.setControl(positionVoltage.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState);

    }

    private void setSpeed(SwerveModuleState desiredState) {
        driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveModuleConstants.kWheelCircumference);
        driveMotor.setControl(driveVelocity);
    }

    public void resetToAbsolute() {
        angleMotor.setPosition(getAngle().getRotations() - angleOffset.getRotations());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), SwerveModuleConstants.kWheelCircumference), 
            Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble()));
    }

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = (Units.inchesToMeters(4.0) / 2) * Math.PI;
    }
}
