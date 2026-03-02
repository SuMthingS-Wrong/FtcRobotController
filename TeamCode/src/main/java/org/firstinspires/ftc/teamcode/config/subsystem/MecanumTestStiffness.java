package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;


@TeleOp
public class MecanumTestStiffness extends OpMode {
    /* instantiate motors */

    private Motor fL;
    private Motor fR;
    private Motor bL;
    private Motor bR;

    private GamepadEx driver1;

    @Override
    public void init() {

        fR = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        fL = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        bR = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);
        bL = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);
        driver1 = new GamepadEx(gamepad1);

        telemetry.addLine("initialised all mechanics");
    }

@Override
    public void loop() {

        // if right trigger, set all motors to 0.5 power
        // otherwise each button individually controls one of the wheels
        if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            fR.motor.setPower(0.5);
            fL.motor.setPower(0.5);
            bR.motor.setPower(0.5);
            bL.motor.setPower(0.5);
        } else if (driver1.getButton(GamepadKeys.Button.A)) {
            fL.motor.setPower(0.5);
        } else if (driver1.getButton(GamepadKeys.Button.B)) {
            fR.motor.setPower(0.5);
        } else if (driver1.getButton(GamepadKeys.Button.X)) {
            bR.motor.setPower(0.5);
        } else if (driver1.getButton((GamepadKeys.Button.Y))) {
            bL.motor.setPower(0.5);
        } else {
            fR.motor.setPower(0);
            fL.motor.setPower(0);
            bR.motor.setPower(0);
            bL.motor.setPower(0);
        }
    }
}
