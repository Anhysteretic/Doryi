package com.anhysteretic.doryi;

import com.anhysteretic.doryi.Vision.Vision;
import com.anhysteretic.doryi.Vision.VisionIO;
import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.constants.RC;
import com.anhysteretic.doryi.drive.Drivetrain;
import com.anhysteretic.doryi.drive.DrivetrainIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Control {

    private double MaxSpeed = RC.whichRobot == RC.WhichRobot.Nike
            ? NikeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            : DoryTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final DrivetrainIO drivetrainIO;
    public final Drivetrain drivetrain;

    public final VisionIO visionIO;
    public final Vision vision;

    public final Command driveCommand;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public Control(DrivetrainIO drivetrainIO, Drivetrain drivetrain, VisionIO visionIO, Vision vision) {
        this.drivetrainIO = drivetrainIO;
        this.drivetrain = drivetrain;
        this.visionIO = visionIO;
        this.vision = vision;

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

    private void configureBindings() {
        drivetrain.setDefaultCommand(driveCommand);

        joystick.start().onTrue(new InstantCommand(drivetrain::seedFieldCentric));
    }
}