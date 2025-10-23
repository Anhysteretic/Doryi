package com.anhysteretic.doryi.Vision;

import com.anhysteretic.doryi.constants.RC;
import com.anhysteretic.doryi.drive.Drivetrain;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Time;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Drivetrain drivetrain;

    public Vision(VisionIO io, Drivetrain drivetrain){
        this.io = io;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        inputs.gyroAngle = drivetrain.getPose().getRotation();
        inputs.robotSpeeds = drivetrain.getChassisSpeeds();
        io.updateInputs(this.inputs);
        Logger.processInputs("Vision", this.inputs);

        Vector<N3> charlieDynamicSTD = RC.Vision.CharlieSTDDevs;
        Vector<N3> gammaDynamicSTD = RC.Vision.GammaSTDDevs;

        if (!(inputs.charlieMegatag2PoseEstimates.fiducialIds == null) && inputs.charlieMegatag2PoseEstimates.fiducialIds.length >= 1){
            Pose2d latestEstimate = inputs.charlieMegatag2PoseEstimates.fieldToCamera;
            double latestEstimateTime = Utils.fpgaToCurrentTime(inputs.charlieMegatag2PoseEstimates.timestampSeconds);
            charlieDynamicSTD = charlieDynamicSTD.times(inputs.charlieMegatag2PoseEstimates.avgTagDist);
            Matrix<N3, N1> finalMatrix = VecBuilder.fill(charlieDynamicSTD.get(0), charlieDynamicSTD.get(1), charlieDynamicSTD.get(2));
            drivetrain.addVisionMeasurement(
                    latestEstimate,
                    latestEstimateTime,
                    finalMatrix
            );
        }

        if (!(inputs.gammaMegatag2PoseEstimates.fiducialIds == null) && inputs.gammaMegatag2PoseEstimates.fiducialIds.length >= 1){
            Pose2d latestEstimate = inputs.gammaMegatag2PoseEstimates.fieldToCamera;
            double latestEstimateTime = Utils.fpgaToCurrentTime(inputs.gammaMegatag2PoseEstimates.timestampSeconds);
            gammaDynamicSTD = gammaDynamicSTD.times(inputs.gammaMegatag2PoseEstimates.avgTagDist);
            Matrix<N3, N1> finalMatrix = VecBuilder.fill(gammaDynamicSTD.get(0), gammaDynamicSTD.get(1), gammaDynamicSTD.get(2));
            drivetrain.addVisionMeasurement(
                    latestEstimate,
                    latestEstimateTime,
                    finalMatrix
            );
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getTimestamp() - timestamp);

        if (RC.robotType == RC.RunType.DEV) {
//            Logger.recordOutput("Vision/CharlieDynamicSTD", charlieDynamicSTD);
//            Logger.recordOutput("Vision/GammaDynamicSTD", gammaDynamicSTD);
        }
    }
}
