// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.anhysteretic.doryi;

import com.anhysteretic.doryi.Vision.Vision;
import com.anhysteretic.doryi.Vision.VisionIO;
import com.anhysteretic.doryi.Vision.VisionIOLimelights;
import com.anhysteretic.doryi.constants.DoryTunerConstants;
import com.anhysteretic.doryi.constants.NikeTunerConstants;
import com.anhysteretic.doryi.drive.Drivetrain;
import com.anhysteretic.doryi.drive.DrivetrainIO;
import com.anhysteretic.doryi.drive.DrivetrainIOReal;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.anhysteretic.doryi.constants.RC;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

//    private final SendableChooser<Command> autoChooser;

    public final DrivetrainIO drivetrainIO;
    public final Drivetrain drivetrain;

    public final VisionIO visionIO;
    public final Vision vision;

    public final Control control;

    public Robot() {

        Logger.recordMetadata("ProjectName", "Doryi");
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (RC.getMode()) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda2/"));
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                break;
            case SIM:
                Logger.addDataReceiver(new RLOGServer());
                break;
            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        switch (RC.robotType){
            default -> {
                if (RC.whichRobot == RC.WhichRobot.Nike) {
                    this.drivetrainIO = new DrivetrainIOReal(
                            NikeTunerConstants.DrivetrainConstants,
                            NikeTunerConstants.FrontLeft,
                            NikeTunerConstants.FrontRight,
                            NikeTunerConstants.BackLeft,
                            NikeTunerConstants.BackRight);
                } else {  //if (RC.whichRobot == RC.WhichRobot.Dory){
                    this.drivetrainIO = new DrivetrainIOReal(
                            DoryTunerConstants.DrivetrainConstants,
                            DoryTunerConstants.FrontLeft,
                            DoryTunerConstants.FrontRight,
                            DoryTunerConstants.BackLeft,
                            DoryTunerConstants.BackRight);
                }

                this.drivetrain = new Drivetrain(this.drivetrainIO);

                this.visionIO = new VisionIOLimelights();
                this.vision = new Vision(visionIO, drivetrain);

                this.control = new Control(drivetrainIO, drivetrain, visionIO, vision);
            }
        }

//        autoChooser = AutoBuilder.buildAutoChooser();
//        SmartDashboard.putData("Auto Chooser", autoChooser);

        SignalLogger.enableAutoLogging(false);
        Logger.start();

        Threads.setCurrentThreadPriority(true, 5);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
//        m_autonomousCommand = autoChooser.getSelected();
        m_autonomousCommand = null;

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
