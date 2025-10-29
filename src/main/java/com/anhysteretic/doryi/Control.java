package com.anhysteretic.doryi;

import com.anhysteretic.doryi.Vision.Vision;
import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.constants.RC;
import com.anhysteretic.doryi.constants.SC;
import com.anhysteretic.doryi.drive.Drivetrain;
import com.anhysteretic.doryi.escalator.Escalator;
import com.anhysteretic.doryi.escalator.MoveCommand;
import com.anhysteretic.doryi.flapSystem.FlapSystem;
import com.anhysteretic.doryi.shooter.Shooter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Control {

    public static double scoreTimeout = 0.5;

    private double MaxSpeed = RC.whichRobot == RC.WhichRobot.Nike
            ? NikeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : DoryTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final Drivetrain drivetrain;
    public final Vision vision;
    private final Escalator escalator;
    private final Shooter shooter;
    private final FlapSystem flapSystem;

    public final Command driveCommand;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Command snapScoreL4;
    public final Command snapScoreL3;
    public final Command snapScoreL2;

    public final Command noSnapAutoScoreL4;
    public final Command noSnapAutoScoreL3;
    public final Command noSnapAutoScoreL2;
    public final Command noSnapAutoScoreL1;

    public final Command escalatorGoHome;

    public final Command intake;

    public Control(Drivetrain drivetrain, Vision vision, Escalator escalator, Shooter shooter, FlapSystem flapSystem) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.escalator = escalator;
        this.shooter = shooter;
        this.flapSystem = flapSystem;

        if (!AutoBuilder.isConfigured()) {
            drivetrain.configureAutoBuilder();
        }


        NamedCommands.registerCommand("L2", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("L3", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L4,
                        this.shooter::shooterHasCoral, this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("AutoL4", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoSnapInline(),
                        new MoveCommand(
                                this.escalator,
                                Escalator.Position.L4,
                                this.shooter::shooterHasCoral,
                                this.shooter::escalatorClear)),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))
                        .withTimeout(1))
                .onlyIf(this.shooter::escalatorClear)
        );
        NamedCommands.registerCommand("GoDown",
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false), escalator));

        NamedCommands.registerCommand("Intake", Intake());

        NamedCommands.registerCommand("PassiveRaise",
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, true)));

        NamedCommands.registerCommand("FlapDown",
                new RunCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.5), flapSystem)
                        .withTimeout(1)
                        .finallyDo(flapSystem::stopFlapper));

        this.snapScoreL4 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL4 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL3 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL2 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL1 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L1,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L1, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.intake = Intake();

        this.escalatorGoHome =
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.HomeAndIntake,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear)
                        .onlyIf(this.shooter::escalatorClear);

        this.driveCommand = Drive(
                this.drivetrain,
                () -> -joystick.getLeftY(),
                () -> -joystick.getLeftX(),
                () -> -joystick.getRightX());

        configureBindings();
    }

    public Command Drive(
            Drivetrain drivetrain,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier angularVelocity) {

        return Commands.run(
                () -> {
                    double controlX = velocityX.getAsDouble();
                    double controlY = velocityY.getAsDouble();
                    double controlAngularVelocity = angularVelocity.getAsDouble();
                    double velX = controlX * MaxSpeed;
                    double velY = controlY * MaxSpeed;
                    double angularVel = controlAngularVelocity * MaxAngularRate;
                    double throttleFieldFrame = RC.isRedAlliance.get() ? -velX : velX;
                    double strafeFieldFrame = RC.isRedAlliance.get() ? -velY : velY;
                    drivetrain.setControl(drivetrain.drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withRotationalRate(angularVel));
                }, drivetrain);
    }

    /**
     * Sequences intaking that runs shooter, passive holds down the elevator, and stops when we have a coral.
     * @return Command
     */
    public Command Intake() {
        return new ConditionalCommand(
                new ParallelRaceGroup(
                        this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                ), new SequentialCommandGroup(
                this.escalator.passiveHoldDown().withTimeout(0.2), new ParallelRaceGroup(
                this.shooter.runShooterIntake(), this.flapSystem.runIntake()
        )

        ).onlyIf(this.shooter::escalatorClear), // Make sure that the elevator is clear before running.
                // move elevator down if not at intaking position
                () -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)
        );
    }

    final double alpha = 1; // 0..1, smoothing (1.0 = no smoothing)

    /**
     * Pose to Pose autoAlign command that will drive to the closest scoring command.
     * It is two ProfiledPIDControllers, one for X and one for Y, that will close the robot pose to the target pose
     * We use SwerveRequest.FieldCentricFacingAngle so that we do not have to deal with rotation
     * First we wait for the headingController to point the robot at the right direction before moving.
     * This deals with issues of profiling turning with vx and vy which is not a simple control.
     * @return AutoAlign Command
     */
    public Command AutoSnapInline(){
        AtomicReference<Pose2d> target = new AtomicReference<>();

        AtomicReference<Vector<N2>> prevSpeeds = new AtomicReference<>(VecBuilder.fill(0, 0));

        return Commands.runOnce(
                // Sets everything up, reset the controllers because we have a non-zero kI
                () -> {

                    target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));

                    ChassisSpeeds drivetrainSpeeds = this.drivetrain.getChassisSpeeds();
                    Translation2d d = target.get().getTranslation().minus(this.drivetrain.getPose().getTranslation());
                    if (drivetrainSpeeds.vxMetersPerSecond == 0 && drivetrainSpeeds.vyMetersPerSecond == 0){
                        drivetrain.snapController.reset(target.get().getTranslation().minus(drivetrain.getPose().getTranslation()).getNorm());
                    } else if (d.getNorm() < 1e-9){
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

                    drivetrain.driveHeading.HeadingController.reset();

                    prevSpeeds.set(VecBuilder.fill(0, 0));

//                    drivetrain.setControl(drivetrain.driveChassisSpeeds.withSpeeds(drivetrainSpeeds));

                    Logger.recordOutput("AutoSnap/PoseTarget", target.get());
                }
        ).andThen(
                Commands.run(
                            () -> {
                                Translation2d d = target.get().getTranslation().minus(this.drivetrain.getPose().getTranslation());
                                double dNorm = d.getNorm();

                                if (dNorm < 1e-6) {
                                    drivetrain.setControl(drivetrain.driveChassisSpeeds.withSpeeds(new ChassisSpeeds()));
                                }

                                double speed = -(drivetrain.snapController.calculate(dNorm, 0)
                                        - drivetrain.snapController.getSetpoint().velocity);

                                Vector<N2> dir = d.toVector().unit();
                                Vector<N2> cmd = dir.times(speed);

                                Vector<N2> smoothed = prevSpeeds.get().times(1 - alpha).plus(cmd.times(alpha));
                                this.drivetrain.setControl(
                                        this.drivetrain.driveHeading
                                                .withVelocityY(smoothed.get(1))
                                                .withVelocityX(smoothed.get(0))
                                                .withTargetDirection(target.get().getRotation())
                                );

                                prevSpeeds.set(smoothed);

                                if (RC.robotType == RC.RunType.DEV){
                                    Logger.recordOutput("AutoSnap/dNorm", dNorm);
                                    Logger.recordOutput("AutoSnap/OutputSpeed", speed);
                                    Logger.recordOutput("AutoSnap/newSpeedsX", smoothed.get(0));
                                    Logger.recordOutput("AutoSnap/newSpeedsY", smoothed.get(1));
                                    Logger.recordOutput("AutoSnap/snapController", drivetrain.snapController.atSetpoint());
                                    Logger.recordOutput("AutoSnap/headingController", drivetrain.driveHeading.HeadingController.atSetpoint());
                                }
                            }
                    ).until(
                            () -> (drivetrain.snapController.atGoal() && drivetrain.driveHeading.HeadingController.atSetpoint())
                    ).finallyDo(
                            () -> this.drivetrain.setControl(
                                    this.drivetrain.driveChassisSpeeds.withSpeeds(new ChassisSpeeds()))
                    )
                );
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(driveCommand);

        joystick.start().onTrue(new InstantCommand(drivetrain::seedFieldCentric));

        /* Zero Automation */
        joystick.leftBumper().and(joystick.y()).whileTrue(noSnapAutoScoreL4);

        joystick.leftBumper().and(joystick.x()).whileTrue(noSnapAutoScoreL3);

        joystick.leftBumper().and(joystick.b()).whileTrue(noSnapAutoScoreL2);

        joystick.leftBumper().and(joystick.a()).whileTrue(noSnapAutoScoreL1);

        joystick.leftBumper().onFalse(escalatorGoHome);

        /* Automated */
        joystick.rightBumper().and(joystick.y()).whileTrue(snapScoreL4);

        joystick.rightBumper().and(joystick.x()).whileTrue(snapScoreL3);

        joystick.rightBumper().and(joystick.b()).whileTrue(snapScoreL2);

        joystick.rightBumper().onFalse(escalatorGoHome);

        /* ###################################### */

        joystick.leftTrigger().and(() -> !this.shooter.shooterHasCoral()).whileTrue(intake);

        joystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Escalator.Position.L4)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL4), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        joystick.rightTrigger(0.1).and(() -> !this.escalator.getTarget().equals(Escalator.Position.L4)).and(() -> !this.escalator.getTarget().equals(Escalator.Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL3L2), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        joystick.rightTrigger(0.1).and(() -> this.escalator.getTarget().equals(Escalator.Position.L1)).whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(SC.Shooter.manualShootDutyCycleL1), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        // joystick.a().whileTrue(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(-0.5), flapSystem)).onFalse(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(0), flapSystem));
        // Zjoystick.a().whileTrue(new InstantCommand(() -> this.shooter.setDutyCycle(-.5), shooter)).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter));

        // joystick.start().onTrue(new InstantCommand(escalator::zero, escalator));

        joystick.povDown().whileTrue(this.escalator.goDown());

        joystick.povUp().whileTrue(this.escalator.goUp());

        joystick.povLeft().whileTrue(
                        new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

        joystick.povRight().whileTrue(
                        new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

//        joystick.rightStick().onTrue(new InstantCommand(() -> this.flapSystem.startClimb(), flapSystem));

        joystick.leftStick().onTrue(new InstantCommand(() -> this.shooter.setDutyCycle(-0.25))).onFalse(new InstantCommand(() -> this.shooter.setDutyCycle(0)));
        joystick.leftStick().onTrue(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(-0.25))).onFalse(new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(0)));
    }
}