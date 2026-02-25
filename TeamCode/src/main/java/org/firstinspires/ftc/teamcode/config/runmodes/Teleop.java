package org.firstinspires.ftc.teamcode.config.runmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.config.subsystem.*;

@TeleOp
public class Teleop extends OpMode {
    private MecanumDrive mecanum;
    private GamepadEx driver1;
    //private Limelight3A limelight;
    private IMU imu;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;

    @Override
    public void init() {
        /* instantiate motors */

        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435)
        );
        driver1 = new GamepadEx(gamepad1);

        /* instantiate limelight */
        //limelight = hardwareMap.get(Limelight3A.class, "limelight"); //change this to device name
        //limelight.pipelineSwitch(0); //april tag change this!
        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void loop() {
        // drive the robot
        mecanum.driveRobotCentric(
                driver1.getLeftX(),
                driver1.getLeftY(),
                driver1.getRightY()
        );


        //check what is pressed on the gamepad
        // if left trigger pressed, auto-rotate the turret
        if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            shooter.shoot();
        };

        // if button a pressed, shoot.
        if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            turret.TurretTracking();
        };
    }

}
