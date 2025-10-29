package com.anhysteretic.doryi.drive;

import com.anhysteretic.doryi.Control;
import com.anhysteretic.doryi.constants.Field;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.constants.RC;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public final SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public final SwerveRequest.FieldCentricFacingAngle driveHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.Position)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDesaturateWheelSpeeds(false);

    public ProfiledPIDController snapController = new ProfiledPIDController(
            5, 0.01, 0.01, new TrapezoidProfile.Constraints(
            4.25, 1.25) // max velocity, max acceleration
    );

    public Drivetrain(DrivetrainIO io) {
        this.io = io;

        this.snapController.setTolerance(Units.inchesToMeters(0.5));

        driveHeading.HeadingController.setPID(10, 0, 0);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveHeading.HeadingController.setTolerance(0.1, 0.1);
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

                double currentSpeed = inputs.ModuleStates[0].speedMetersPerSecond;
                double targetSpeed = inputs.ModuleTargets[0].speedMetersPerSecond;

                Logger.recordOutput("Drivetrain/currentSpeed", currentSpeed);
                Logger.recordOutput("Drivetrain/DriveTargetSpeed", targetSpeed);

                Logger.recordOutput("Drivetrain/Current", io.getDriveMotor().getTorqueCurrent().getValue());
                Logger.recordOutput("Drivetrain/MotorKT", io.getDriveMotor().getMotorKT().getValue());

                ChassisSpeeds fieldCentric = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
                Logger.recordOutput("Drivetrain/SpeedMag", Math.hypot(fieldCentric.vxMetersPerSecond, fieldCentric.vyMetersPerSecond));

                Logger.recordOutput("Drivetrain/HeadingControllerSetpoint", this.driveHeading.HeadingController.getSetpoint());
                Logger.recordOutput("Drivetrain/HeadingControllerPosition", this.getPose().getRotation().getRadians());
                Logger.recordOutput("Drivetrain/HeadingControllerAtSetpoint", this.driveHeading.HeadingController.atSetpoint());
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

    /**
     * Run calculation to the reefpose to find the polar angle the robot is to the reef in wpilib coordinate system
     * @return double
     */
    public double getAngleToReefPolar() {
        boolean isRed = RC.isRedAlliance.get();
        Translation2d robotVector;

        if (isRed) robotVector = this.getPose().getTranslation().minus(Field.Positions.Reef.redTranslation2d);
        else robotVector = this.getPose().getTranslation().minus(Field.Positions.Reef.blueTranslation2d);

        return Math.atan2(robotVector.getY(), robotVector.getX()) * 57.2957795131;
    }

    /**
     * Calculate the scoring letter the robot is closet to based of the angle to the reef it is at
     * @param angle the angle in degrees to determine the scoring position
     * @return Field.ScoringPositions
     */
    public Field.ScoringPositions getScoringPosition(double angle) {
        if (angle >= 0 && angle < 30) {
            return Field.ScoringPositions.H;
        } else if (angle >= 30 && angle < 60) {
            return Field.ScoringPositions.I;
        } else if (angle >= 60 && angle < 90) {
            return Field.ScoringPositions.J;
        } else if (angle >= 90 && angle < 120) {
            return Field.ScoringPositions.K;
        } else if (angle >= 120 && angle < 150) {
            return Field.ScoringPositions.L;
        } else if (angle >= 150 && angle < 180) {
            return Field.ScoringPositions.A;
        } else if (angle >= -180 && angle < -150) {
            return Field.ScoringPositions.B;
        } else if (angle >= -150 && angle < -120) {
            return Field.ScoringPositions.C;
        } else if (angle >= -120 && angle < -90) {
            return Field.ScoringPositions.D;
        } else if (angle >= -90 && angle < -60) {
            return Field.ScoringPositions.E;
        } else if (angle >= -60 && angle < -30) {
            return Field.ScoringPositions.F;
        } else {
            return Field.ScoringPositions.G;
        }
    }

    public Pose2d getPoseToScore(double angle) {
        if (RC.isRedAlliance.get()) {
            return getScoringPosition(angle).getPoseRed();
        } else {
            return getScoringPosition(angle).getPoseBlue();
        }
    }

    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> inputs.Pose, // Supplier of current robot pose
                    io::seedFieldRelative, // Consumer for seeding pose against auto
                    () -> inputs.Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> io.setControl(
                            driveChassisSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(0.6, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(2.75, 0, 0)), config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
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
