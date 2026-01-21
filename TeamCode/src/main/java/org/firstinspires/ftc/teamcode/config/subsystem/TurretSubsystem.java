package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class TurretSubsystem extends SubsystemBase {
    private final Servo LeftTurretServo;
    private final Servo RightTurretServo;

    public TurretSubsystem(final HardwareMap hMap) {
        LeftTurretServo = hMap.get(Servo.class, "leftturretservo");
        RightTurretServo = hMap.get(Servo.class, "rightturretservo");
    }

    public void SetPosition() {

    }

    public void TurretTracking() {

    }
}
