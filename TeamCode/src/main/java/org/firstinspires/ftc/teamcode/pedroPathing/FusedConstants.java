package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FusedConstants {
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private Pose currentPose = new Pose(0,0,0);
    private Pose startPose = new Pose(0,0,0);

    public FusedConstants(HardwareMap hardwareMap) {
        // Initialize Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-50.0, 120.0, DistanceUnit.MM); // SET YOUR OFFSETS HERE (mm)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Ensure using AprilTag pipeline
        limelight.start();
    }
}
