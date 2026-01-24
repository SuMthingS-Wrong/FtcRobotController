package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem extends SubsystemBase {
    private final Servo LeftTurretServo, RightTurretServo;
    private final Limelight3A limelight;



    private static final double K_P = 0.005;
    private static final double CAMERA_HEIGHT = 10.0;
    private static final double TARGET_HEIGHT = 6.0;
    private static final double MOUNT_ANGLE = 20.0;

    private double currentPosition = 0.5;
    private double targetDistance = 0;
    private double targetAngle = 0;
    private boolean targetVisible = false;

    public TurretSubsystem(final HardwareMap hMap) {
        LeftTurretServo = hMap.get(Servo.class, "leftturretservo");
        RightTurretServo = hMap.get(Servo.class, "rightturretservo");
        RightTurretServo.setDirection(Servo.Direction.REVERSE);

        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public void SetPosition() {

    }

    public void TurretTracking() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            targetVisible = true;
            double tx = result.getTx();
            double ty = result.getTy();

            // 1. Calculate Angle and Distance
            targetAngle = tx;
            double angleRad = Math.toRadians(MOUNT_ANGLE + ty);
            targetDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            // 2. Tracking Logic (PID)
            // If tx is positive, target is to the right, so increase servo position
            double error = tx;
            currentPosition += (error * K_P);
            currentPosition = Range.clip(currentPosition, 0.0, 1.0);

            LeftTurretServo.setPosition(currentPosition);
            RightTurretServo.setPosition(currentPosition);
        } else {
            targetVisible = false;
        }
    }

    public double getDistance() { return targetDistance; }
    public double getAngle() { return targetAngle; }
    public boolean hasTarget() { return targetVisible; }

}
