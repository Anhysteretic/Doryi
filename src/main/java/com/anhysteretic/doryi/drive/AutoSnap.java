package com.anhysteretic.doryi.drive;

import com.anhysteretic.doryi.constants.RC;
import com.anhysteretic.doryi.escalator.Escalator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public class AutoSnap extends Command {

    AtomicReference<Pose2d> target = new AtomicReference<>();
    AtomicReference<Vector<N2>> prevSpeeds = new AtomicReference<>(VecBuilder.fill(0, 0));

    Drivetrain drivetrain;

    public AutoSnap(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
        setName("AutoSnap");
    }

    public void reTarget(){
        target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));
        prevSpeeds.set(VecBuilder.fill(0, 0));
        ChassisSpeeds drivetrainSpeeds = this.drivetrain.getChassisSpeeds();
        Translation2d d = target.get().getTranslation().minus(this.drivetrain.getPose().getTranslation());
        if (drivetrainSpeeds.vxMetersPerSecond == 0 && drivetrainSpeeds.vyMetersPerSecond == 0) {
            drivetrain.snapController.reset(target.get().getTranslation().minus(drivetrain.getPose().getTranslation()).getNorm());
        } else if (d.getNorm() < 1e-9) {
            this.drivetrain.snapController.reset(0);
        } else {
            drivetrainSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drivetrainSpeeds, drivetrain.getPose().getRotation());
            Translation2d v = new Translation2d(drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond);
            Vector<N2> projection = v.toVector().projection(d.toVector());
            this.drivetrain.snapController.reset(
                    target.get().getTranslation().minus(drivetrain.getPose().getTranslation()).getNorm(),
                    -projection.norm()
            );
        }

        drivetrain.rotationController.reset();
        Logger.recordOutput("AutoSnap/PoseTarget", target.get());
    }

    @Override
    public void initialize() {
        reTarget();
    }

    @Override
    public void execute() {
        Translation2d d = target.get().getTranslation().minus(this.drivetrain.getPose().getTranslation());
        double dNorm = d.getNorm();

        if (dNorm < 1e-6) {
            drivetrain.setControl(drivetrain.driveChassisSpeeds.withSpeeds(new ChassisSpeeds()));
        }

        double speed = -(drivetrain.snapController.calculate(dNorm, 0)
                - drivetrain.snapController.getSetpoint().velocity);

        Vector<N2> dir = d.toVector().unit();
        Vector<N2> cmd = dir.times(speed);

        Vector<N2> smoothed = cmd;

        double rotationOutput =
                drivetrain.rotationController.calculate(
                        this.drivetrain.getPose().getRotation().getRadians(),
                        target.get().getRotation().getRadians()) * 2 * Math.PI;

        this.drivetrain.setControl(
                this.drivetrain.drive
                        .withVelocityY(smoothed.get(1))
                        .withVelocityX(smoothed.get(0))
                        .withRotationalRate(rotationOutput)
        );

        prevSpeeds.set(smoothed);

        if (RC.robotType == RC.RunType.DEV) {
            Logger.recordOutput("AutoSnap/dNorm", dNorm);
            Logger.recordOutput("AutoSnap/OutputSpeed", speed);
            Logger.recordOutput("AutoSnap/newSpeedsX", smoothed.get(0));
            Logger.recordOutput("AutoSnap/newSpeedsY", smoothed.get(1));
            Logger.recordOutput("AutoSnap/snapController", drivetrain.snapController.atSetpoint());
            Logger.recordOutput("AutoSnap/headingController", drivetrain.driveHeading.HeadingController.atSetpoint());
        }
    }

    /**
     * Determines if the command is finished. If the elevator is not clear, then the command also finishes.
     * @return true if the elevator is not clear or at the position
     */
    @Override
    public boolean isFinished() {
        return drivetrain.snapController.atGoal() && drivetrain.rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setControl(
                this.drivetrain.driveChassisSpeeds.withSpeeds(new ChassisSpeeds()));
    }
}
