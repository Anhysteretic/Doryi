package com.anhysteretic.doryi;

import com.anhysteretic.doryi.Vision.Vision;
import com.anhysteretic.doryi.Vision.VisionIO;
import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.constants.RC;
import com.anhysteretic.doryi.constants.SC;
import com.anhysteretic.doryi.drive.Drivetrain;
import com.anhysteretic.doryi.drive.DrivetrainIO;
import com.anhysteretic.doryi.escalator.Escalator;
import com.anhysteretic.doryi.escalator.MoveCommand;
import com.anhysteretic.doryi.flapSystem.FlapSystem;
import com.anhysteretic.doryi.shooter.Shooter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    /**
     * Pose to Pose autoAlign command that will drive to the closest scoring command.
     * It is two ProfiledPIDControllers, one for X and one for Y, that will close the robot pose to the target pose
     * We use SwerveRequest.FieldCentricFacingAngle so that we do not have to deal with rotation
     * First we wait for the headingController to point the robot at the right direction before moving.
     * This deals with issues of profiling turning with vx and vy which is not a simple control.
     * @return AutoAlign Command
     */
    public Command AutoSnapInline(){
        ProfiledPIDController vx = drivetrain.vxController;
        ProfiledPIDController vy = drivetrain.vyController;
        AtomicReference<Pose2d> target = new AtomicReference<>();

        return Commands.runOnce(
                // Sets everything up, reset the controllers because we have a non-zero kI
                () -> {
                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                    target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));
                    Logger.recordOutput("AutoSnap/PoseTarget", target.get());
                    Translation2d translation2d = this.drivetrain.getPose().getTranslation();
                    vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
                    vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
                }
        ).andThen(
                Commands.run(
                        // Points the drivetrain in the right direction
                        () -> this.drivetrain.setControl(
                                drivetrain.driveHeading.withTargetDirection(target.get().getRotation()))
                ).until(
                        // Wait until we are facing the right direction
                        this.drivetrain.driveHeading.HeadingController::atSetpoint
                ).andThen(
                        Commands.run(
                                () -> {
                                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                            this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                                    Translation2d translation2d = this.drivetrain.getPose().getTranslation();

                                    // Add the current setpoint velocity to the new velocity so that it is smoother.
                                    double outputX =
                                            vx.calculate(translation2d.getX(), target.get().getX()) +
                                                    vx.getSetpoint().velocity;
                                    double outputY =
                                            vy.calculate(translation2d.getY(), target.get().getY()) +
                                                    vy.getSetpoint().velocity;

                                    // If it is not scaled, then each controller can output speeds that
                                    // when added up to the final vector will exceed the maximum real speed of the robot.
                                    outputX *= 0.7;
                                    outputY *= 0.7;

                                    // Do some logging to help with debugging
                                    Logger.recordOutput("AutoSnap/outputX", outputX);
                                    Logger.recordOutput("AutoSnap/outputY", outputY);
                                    Logger.recordOutput("AutoSnap/vxAtGoal", vx.atGoal());
                                    Logger.recordOutput("AutoSnap/vyAtGoal", vy.atGoal());

                                    this.drivetrain.setControl(
                                            this.drivetrain.driveHeading
                                                    .withVelocityY(outputY)
                                                    .withVelocityX(outputX)
                                                    .withTargetDirection(target.get().getRotation())
                                    );
                                }
                        ).until(
                                () -> vx.atGoal() && vy.atGoal()
                        ).finallyDo(
                                () -> this.drivetrain.setControl(
                                        this.drivetrain.driveChassisSpeeds.withSpeeds(new ChassisSpeeds()))
                        )
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