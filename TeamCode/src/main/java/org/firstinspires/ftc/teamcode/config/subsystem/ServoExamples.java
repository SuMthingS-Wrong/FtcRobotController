package org.firstinspires.ftc.teamcode.config.subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class ServoExamples extends OpMode {
    ServoTest bench  = new ServoTest();
    private DcMotor intakeMotor;
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //change this to device name
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");//april tag change this!
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        bench.init(hardwareMap);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llresult = limelight.getLatestResult();
        double angle = llresult.getTx()/360;
        bench.setServoPos(angle);
        if (gamepad1.left_trigger>0) {
            intakeMotor.setPower(-1.2);
        } else {
            intakeMotor.setPower(0);
        }

    }
}
