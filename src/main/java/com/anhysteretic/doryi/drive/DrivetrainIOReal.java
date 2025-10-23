package com.anhysteretic.doryi.drive;

import com.anhysteretic.doryi.constants.RC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DrivetrainIOReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO {

    AtomicReference<SwerveDrivetrain.SwerveDriveState> telemetryCache_ = new AtomicReference<>();
    Consumer<SwerveDrivetrain.SwerveDriveState> telemetry_ = swerveDriveState -> telemetryCache_.set(swerveDriveState);

    public DrivetrainIOReal(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules){
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);

        this.registerTelemetry(telemetry_);
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs){
        inputs.fromSwerveDriveState(telemetryCache_.get());
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs){
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void pointModulesAtAngle() {
        this.setControl(new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.kZero));
    }

    @Override
    public void seedFieldCentric() {
        super.seedFieldCentric();
    }

    /**
     * Sets the odometry pose
     */
    @Override
    public void seedFieldRelative(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return new RunCommand(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }
}
