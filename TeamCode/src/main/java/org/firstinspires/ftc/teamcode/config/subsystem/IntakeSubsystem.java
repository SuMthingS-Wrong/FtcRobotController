package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intake1;
    private final DcMotorEx intake2;

    public IntakeSubsystem(final HardwareMap hMap, final String name) {
        intake1 = hMap.get(DcMotorEx.class, name);
        intake2 = hMap.get(DcMotorEx.class, name);
    }

    public void on() {

    }

    public void off() {

    }
}
