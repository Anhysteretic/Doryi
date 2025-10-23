package com.anhysteretic.doryi.Vision;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs{
        public boolean charlieSeesTarget;
        public boolean gammaSeesTarget;

        public FiducialObservation[] charlieFiducialObservations = new FiducialObservation[0];
        public FiducialObservation[] gammaFiducialObservations = new FiducialObservation[0];

        public MegatagPoseEstimate charlieMegatagPoseEstimate = new MegatagPoseEstimate();
        public MegatagPoseEstimate gammaMegatagPoseEstimate = new MegatagPoseEstimate();

        public int charlieMegatagCount;
        public int gammaMegatagCount;

        public MegatagPoseEstimate charlieMegatag2PoseEstimates = new MegatagPoseEstimate();
        public MegatagPoseEstimate gammaMegatag2PoseEstimates = new MegatagPoseEstimate();

        public Rotation2d gyroAngle = Rotation2d.kZero;
        public ChassisSpeeds robotSpeeds = new ChassisSpeeds();
    }

    void updateInputs(VisionIOInputs inputs);
}
