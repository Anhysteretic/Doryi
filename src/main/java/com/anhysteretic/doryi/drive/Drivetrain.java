package com.anhysteretic.doryi.drive;

import com.anhysteretic.doryi.Control;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.constants.RC;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase {
    DrivetrainIO io;
    DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
    Double MaxSpeed = RC.whichRobot == RC.WhichRobot.Nike
            ? NikeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : DoryTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    Telemetry telemetry = new Telemetry(MaxSpeed);

    public final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(Control.MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);
        Logger.recordOutput("Drivetrain/latencyPeriodicSec", Timer.getTimestamp() - timestamp);

        if (RC.robotType == RC.RunType.DEV) {
            if (inputs.ModuleStates != null) {
                telemetry.telemeterize(inputs);

                double currentRad = inputs.ModuleStates[0].angle.getRadians();
                double targetRad = inputs.ModuleTargets[0].angle.getRadians();

                Logger.recordOutput("Drivetrain/TargetAngle", wrapp(targetRad));
                Logger.recordOutput("Drivetrain/DriveCurrentAngle", wrapp(currentRad));
            }
        }
    }

    static double wrapp(double radians){
        double twopi = 2 * Math.PI;
        radians = radians % twopi;
        if (radians < 0) {
            radians += twopi;
        }

        if (radians <= Math.PI){
            return radians;
        } else {
            return twopi - radians;
        }
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    // HELPERS \\\\\\\\\\\\\\\\\\
    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public Pose2d getPose() {
        return inputs.Pose;
    }

    public void seedFieldCentric() {
        io.seedFieldCentric();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return inputs.Speeds;
    }

    public void setPose(Pose2d pose) {
        io.seedFieldRelative(pose);
    }

    public void alignModule() {
        io.pointModulesAtAngle();
    }
}
