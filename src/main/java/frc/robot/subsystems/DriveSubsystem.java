package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem.SwerveConstants.BackLeft;
import frc.robot.subsystems.DriveSubsystem.SwerveConstants.BackRight;
import frc.robot.subsystems.DriveSubsystem.SwerveConstants.FrontLeft;
import frc.robot.subsystems.DriveSubsystem.SwerveConstants.FrontRight;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private Pigeon2 gyro;
    private SwerveDriveOdometry odometry;

    private final PIDController translationController = new PIDController(0, 0, 0);
    private final ProfiledPIDController rotationController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(SwerveConstants.kMaxRotationSpeed.getRadians(), 2));

    public Field2d field;
    
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d[]{
            FrontLeft.kTranslation, 
            FrontRight.kTranslation, 
            BackLeft.kTranslation, 
            BackRight.kTranslation
        });

    public DriveSubsystem() {
        frontLeft = new SwerveModule(
            FrontLeft.kDriveMotorID, 
            FrontLeft.kAngleMotorID, 
            FrontLeft.kEncoderID, 
            FrontLeft.kAngleOffset
            );
        frontRight = new SwerveModule(
            FrontRight.kDriveMotorID, 
            FrontRight.kAngleMotorID, 
            FrontRight.kEncoderID, 
            FrontRight.kAngleOffset
            );
        backLeft = new SwerveModule(
            BackLeft.kDriveMotorID, 
            BackLeft.kAngleMotorID, 
            BackLeft.kEncoderID, 
            BackLeft.kAngleOffset
            );
        backRight = new SwerveModule(
            BackRight.kDriveMotorID, 
            BackRight.kAngleMotorID, 
            BackRight.kEncoderID, 
            BackRight.kAngleOffset
            );
        gyro = new Pigeon2(SwerveConstants.kGyroID);
        odometry = new SwerveDriveOdometry(kinematics, 
            Rotation2d.fromDegrees(gyro.getAngle()), 
            getModulePositions());

        field = new Field2d();
        SmartDashboard.putData(field);
    }

    public Command teleopDrive(double forward, double left, double rotate, boolean fieldOriented) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(MathUtil.clamp(forward, -1.0, 1.0) * SwerveConstants.kMaxSpeed, 
                MathUtil.clamp(left, -1.0, 1.0) * SwerveConstants.kMaxSpeed, 
                MathUtil.clamp(rotate, -1.0, 1.0) * SwerveConstants.kMaxRotationSpeed.getRadians());
            if(fieldOriented) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.getAngle()));
            }
            SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
            setModuleStates(desiredStates);
        });
    }

    public Command teleopDrive(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier rotate) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(MathUtil.clamp(forward.getAsDouble(), -1.0, 1.0) * SwerveConstants.kMaxSpeed, 
                MathUtil.clamp(left.getAsDouble(), -1.0, 1.0) * SwerveConstants.kMaxSpeed, 
                MathUtil.clamp(rotate.getAsDouble(), -1.0, 1.0) * SwerveConstants.kMaxRotationSpeed.getRadians());
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.getAngle()));
            SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
            setModuleStates(desiredStates);
        });
    }

    public Command pidToPose(Pose2d targetPose) {
        return run(() -> {
            teleopDrive(
                translationController.calculate(getPose2d().getX(), targetPose.getX()),
                translationController.calculate(getPose2d().getY(), targetPose.getY()),
                rotationController.calculate(gyro.getAngle() % 360, targetPose.getRotation().getDegrees()),
                true);
        }).until(() -> {
            Pose2d error = targetPose.relativeTo(getPose2d());
            return Math.abs(error.getX()) < .1 
                && Math.abs(error.getY()) < .1 
                && Math.abs(targetPose.getRotation().getDegrees() - gyro.getAngle()) < 5.0;
        }).finallyDo(() -> teleopDrive(0, 0, 0, false));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setModuleState(desiredStates[0]);
        frontRight.setModuleState(desiredStates[1]);
        backLeft.setModuleState(desiredStates[2]);
        backRight.setModuleState(desiredStates[3]);
    }

    public void updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), getModulePositions());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getCurrentPosition(),
            frontRight.getCurrentPosition(),
            backLeft.getCurrentPosition(),
            backRight.getCurrentPosition()
        };
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void periodic() {
        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());
        System.out.println(odometry.getPoseMeters());
    }

    public static final class SwerveConstants {
        public static final class FrontLeft {
            public static final Translation2d kTranslation = new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(13));
            public static final int kDriveMotorID = 6;
            public static final int kAngleMotorID = 7;
            public static final int kEncoderID = 8;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0);
        }

        public static final class FrontRight {
            public static final Translation2d kTranslation = new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(-13));
            public static final int kDriveMotorID = 9;
            public static final int kAngleMotorID = 10;
            public static final int kEncoderID = 11;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0);
        }

        public static final class BackLeft {
            public static final Translation2d kTranslation = new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13));
            public static final int kDriveMotorID = 12;
            public static final int kAngleMotorID = 13;
            public static final int kEncoderID = 14;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0);
        }

        public static final class BackRight {
            public static final Translation2d kTranslation = new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13));
            public static final int kDriveMotorID = 15;
            public static final int kAngleMotorID = 16;
            public static final int kEncoderID = 17;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0);
        }

        public static final int kGyroID = 18;
        public static final double kMaxSpeed = Units.feetToMeters(17.1);
        public static final Rotation2d kMaxRotationSpeed = Rotation2d.fromDegrees(720.0);
    }
}
