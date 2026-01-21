package org.firstinspires.ftc.teamcode.config.runmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    private Motor fL, fR, bL, bR;
    private MecanumDrive mecanum;
    private GamepadEx driver1;

    @Override
    public void init() {
        /* instantiate motors */

        mecanum = new MecanumDrive(fL, fR, bL, bR);
        driver1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        mecanum.driveRobotCentric(
                driver1.getLeftX(),
                driver1.getLeftY(),
                driver1.getRightY()
        );
    }

}
