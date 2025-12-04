package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LancersBotConfig;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

public class LimelightWrapper {

    private final Limelight3A limelight;

    public LimelightWrapper(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LancersBotConfig.LIMELIGHT);
        limelight.pipelineSwitch(0); // TODO: Change this pipeline to the correct pipeline later
        limelight.start(); // Start polling
    }

    /**
     * Returns the robot pose as a RoadRunner Pose2d (in inches, heading in radians)
     * Returns null if no valid fiducial/pose data
     */
    public Pose2d getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()){ // should only return a value if it can find april tags
            return null;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        boolean validTagSeen = false;

        for (LLResultTypes.FiducialResult fr : tags) {
            int id = fr.getFiducialId();

            if (id == 20 || id == 24) {
                validTagSeen = true;
                break;
            }
        }

        if (!validTagSeen) {
            return null;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null){
            return null;
        }
        double xIn = botpose.getPosition().x * 39.37; // convert from meters to inches
        double yIn = botpose.getPosition().y * 39.37;
        double headingRad = Math.toRadians(botpose.getOrientation().getYaw()); // rotation around vertical axis
        return new Pose2d(xIn, yIn, headingRad);
    }

    public void stop() {
        limelight.stop();
    }
}
