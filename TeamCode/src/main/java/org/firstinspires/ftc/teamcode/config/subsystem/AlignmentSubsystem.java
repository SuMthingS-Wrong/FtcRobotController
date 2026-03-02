package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AlignmentSubsystem extends SubsystemBase {

    // ---------------- PD Controller -------------- //

    double kP = 0.001;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.2;
    double kD = 0.00001;
    double curTime = 0;
    double lastTime = 0;

    // -------------- Driving setup --------------- //
    double forward, strafe, rotate;


    public void init() {

    }

    public void start() {
    }


    public void update(LLResult llresult, HardwareMap hwmap) {

    }
}
