package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;



import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeTest extends OpMode {

    private Motor intake;
    private GamepadEx driver1;

    @Override
    public void init() {
        /* instantiate motors */

        intake = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        driver1 = new GamepadEx(gamepad1);

        /* instantiate limelight */
        telemetry.addLine("initialised all mechanics");
    }

    @Override
    public void loop() {
        intake.motor.setPower(driver1.getLeftY());
    }


}
