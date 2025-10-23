package com.anhysteretic.doryi.Vision;

import com.anhysteretic.doryi.constants.RC;
import com.limelight.LimelightHelpers;
import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelights implements VisionIO {
    NetworkTable CharlieTable = NetworkTableInstance.getDefault().getTable(RC.Vision.LimelightCharlieName);
    NetworkTable GammaTable = NetworkTableInstance.getDefault().getTable(RC.Vision.LimelightGammaName);

    VisionIOInputs inputCache = new VisionIOInputs();

    public VisionIOLimelights() {
        setLLSettings();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputCache = inputs;
        setLLSettings();

        inputs.gammaSeesTarget = GammaTable.getEntry("tv").getDouble(0) == 1.0;
        inputs.charlieSeesTarget = CharlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.gammaSeesTarget){
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightGammaName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightGammaName);
            if (megatag != null && megatag2 != null) {
                inputs.gammaMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                inputs.gammaMegatagCount = megatag.tagCount;
                inputs.gammaMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
                inputs.gammaFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
            }
        }

        if (inputs.charlieSeesTarget) {
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightCharlieName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightCharlieName);
            if (megatag != null && megatag2 != null) {
                inputs.charlieMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                inputs.charlieMegatagCount = megatag.tagCount;
                inputs.charlieMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
                inputs.charlieFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
            }
        }
    }

    private void setLLSettings() {
        GammaTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Vision.GammaPose);
        CharlieTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Vision.CharliePose);

        Rotation2d gyroAngle = inputCache.gyroAngle;
        double gyroAngularVelocity = Math.toDegrees(inputCache.robotSpeeds.omegaRadiansPerSecond);
        try {
            LimelightHelpers.SetIMUMode(RC.Vision.LimelightGammaName, 1);
            LimelightHelpers.SetIMUMode(RC.Vision.LimelightCharlieName, 1);
            LimelightHelpers.SetRobotOrientation(
                    RC.Vision.LimelightGammaName,
                    gyroAngle.getDegrees(), gyroAngularVelocity, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(
                    RC.Vision.LimelightCharlieName,
                    gyroAngle.getDegrees(), gyroAngularVelocity, 0, 0, 0, 0);

        } catch (Exception e) {
            System.out.println("failed to log ll settings");
        }
    }
}

