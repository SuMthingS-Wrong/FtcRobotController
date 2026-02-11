package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;

    private final Servo releaseservo;

    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = hMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hMap.get(DcMotorEx.class, "shooter2");
        releaseservo = hMap.get(Servo.class, "ReleaseServo");
    }

    public void shoot() {
        // get distance from april tag

        // convert to angle

        // move to angle

        // convert to power

        // apply this power
    }

    public void spoolup() {

    }
}
