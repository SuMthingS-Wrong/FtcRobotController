package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedConstants;

// GoBilda Pinpoint Driver
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public abstract class FusedLocalizer implements Localizer {

    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private Pose currentPose = new Pose(0,0,0);
    private Pose startPose = new Pose(0,0,0);


    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        Vector v = new Vector(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH));
        return new Pose(v.getXComponent(), v.getYComponent(), odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH));
    }

    @Override
    public void setStartPose(Pose pose) {
        this.startPose = pose;
        odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()
        ));
    }

    @Override
    public void setPose(Pose pose) {
        setStartPose(pose);
    }

    @Override
    public void update() {
        odo.update();

        // 1. Get Pinpoint Data (High Frequency, Smooth)
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D odoPose = odo.getPosition();
        double odoHeading = odo.getHeading(AngleUnit.RADIANS);

        // 2. Feed Heading to Limelight for MegaTag2
        // MegaTag2 REQUIRES valid heading to calculate X/Y accurately
        limelight.updateRobotOrientation(Math.toDegrees(odoHeading));

        LLResult result = limelight.getLatestResult();

        // 3. Fusion Logic
        // Only correct if we see tags and the data is trustworthy
        if (result != null && result.isValid()) {
            // Get MegaTag2 Pose (Field Space)
            Pose3D mt2Pose = result.getBotpose_MT2();

            // Check confidence (e.g., seen multiple tags or close range)
            if (mt2Pose != null) {
                // Convert to Pedro Pose (Inches)
                double visX = mt2Pose.getPosition().x * 39.37; // Meters to Inches
                double visY = mt2Pose.getPosition().y * 39.37;

                // Simple Fusion: If distance between Odo and Vision is significant but not insane,
                // gently reset Pinpoint to match Vision (prevents drift).
                // In a real match, you might simply rely on Pinpoint and use Vision only to "Reset"
                // via a specific command, but here is an automatic approach:

                double diffX = Math.abs(visX - odoPose.getX(DistanceUnit.INCH));
                double diffY = Math.abs(visY - odoPose.getY(DistanceUnit.INCH));

                // If drift is > 1 inch but < 10 inches, correct it.
                if (diffX > 1.0 && diffX < 10.0 || diffY > 1.0 && diffY < 10.0) {
                    // We trust Limelight for X/Y, but keep Pinpoint for Heading
                    odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                            DistanceUnit.INCH, visX, visY, AngleUnit.RADIANS, odoHeading
                    ));
                }
            }
        }

        // Update local cache
        odoPose = odo.getPosition(); // Re-read in case we updated it
        currentPose = new Pose(odoPose.getX(DistanceUnit.INCH), odoPose.getY(DistanceUnit.INCH), odoPose.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public double getTotalHeading() {
        return currentPose.getHeading();
    }

    @Override
    public double getIMUHeading() {
        return currentPose.getHeading();
    }
    public double getForwardMultiplier() {
        return 1.0; // Change this if the robot moves 10" but odo says 11"
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0; // Same for strafing
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0; // Same for rotating
    }

}