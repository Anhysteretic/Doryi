package com.anhysteretic.doryi.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;

public interface DrivetrainIO {

    @AutoLog
    class DrivetrainIOInputs extends SwerveDrivetrain.SwerveDriveState{
        public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState inputState){
            this.FailedDaqs = inputState.FailedDaqs;
            this.ModuleStates = inputState.ModuleStates;
            this.ModulePositions = inputState.ModulePositions;
            this.ModuleTargets = inputState.ModuleTargets;
            this.OdometryPeriod = inputState.OdometryPeriod;
            this.Pose = inputState.Pose;
            this.RawHeading = inputState.RawHeading;
            this.Speeds = inputState.Speeds;
            this.SuccessfulDaqs = inputState.SuccessfulDaqs;
            this.Timestamp = inputState.Timestamp;
        }
    }

    void updateInputs(DrivetrainIOInputs inputs);

    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);

    void seedFieldCentric();

    void seedFieldRelative(Pose2d pose);

    Pose2d getPose();

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void setOperatorPerspectiveForward(Rotation2d fieldDirection);

    void pointModulesAtAngle();

    TalonFX getDriveMotor();
}
