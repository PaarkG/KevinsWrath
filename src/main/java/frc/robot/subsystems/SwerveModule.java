package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Conversions;

public class SwerveModule {
    private TalonFX driveMotor, angleMotor;
    private CANcoder angleEncoder;

    private Rotation2d angleOffset;

    private VelocityVoltage driveVelocity;
    private PositionVoltage positionVoltage;

    private Slot0Configs driveConfigs;
    private Slot0Configs angleConfigs;
    
    public SwerveModule(int driveID, int steerID, int encoderID, Rotation2d angleOffset) {
        driveMotor = new TalonFX(driveID);
        angleMotor = new TalonFX(steerID);
        angleEncoder = new CANcoder(encoderID);
        this.angleOffset = angleOffset;

        driveVelocity = new VelocityVoltage(0).withSlot(0);
        positionVoltage = new PositionVoltage(0).withSlot(0);
        
        driveConfigs = new Slot0Configs();
        driveConfigs.kP = SwerveModuleConstants.kDriveP;
        driveMotor.getConfigurator().apply(driveConfigs);

        angleConfigs = new Slot0Configs();
        angleConfigs.kP = SwerveModuleConstants.kAngleP;
        angleMotor.getConfigurator().apply(angleConfigs);
    }

    public void setModuleState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentState().angle);
        angleMotor.setControl(positionVoltage.withPosition(desiredState.angle.getRotations()).withSlot(0));
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        driveMotor.setControl(driveVelocity.withVelocity(Conversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveModuleConstants.kWheelCircumference)));
    }

    public void resetToAbsolute() {
        angleMotor.setPosition(getAngle().getRotations() - angleOffset.getRotations());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), SwerveModuleConstants.kWheelCircumference), 
            getAngle());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
            Conversions.RPSToMPS(driveMotor.getPosition().getValueAsDouble(), SwerveModuleConstants.kWheelCircumference), 
            getAngle());
    }

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = (Units.inchesToMeters(4.0) / 2) * Math.PI;
        public static final double kDriveP = 0.05;
        public static final double kAngleP = 0.05;
    }
}
