package org.firstinspires.ftc.teamcode.config.runmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.config.subsystem.*;

@TeleOp
public class Teleop extends OpMode {
    private GamepadEx driver1;
    private Limelight3A limelight;
    private ShooterSubsystem shooter;
    private AlignmentSubsystem alignment;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    private DcMotor intakeMotor;


    // ---------------- PD Controller -------------- //

    double kP = 0.02;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.2;
    double kD = 0.005;
    double currTime = 0;
    double lastTime = 0;


    @Override
    public void init() {

        /* instantiate motors */

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* instantiate driver/gamepad */
        driver1 = new GamepadEx(gamepad1);

        /* instantiate limelight */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //april tag change this!

        /* instantiate alignment subsystem */
        alignment = new AlignmentSubsystem();

        /* instantiate shooter */
        //shooter = new ShooterSubsystem(hardwareMap);

        /* instantiate intake motor */
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        telemetry.addLine("initialised all mechanics");
    }

    public void start() {
        limelight.start();
        resetRuntime();
        currTime = getRuntime();
    }

    @Override
    public void loop() {

        /*
         *
         * ================
         *   LIMELIGHT READING
         * ================
         *
         */

        // get limelight reading
        LLResult llresult = limelight.getLatestResult();
        if (llresult.isValid()) {
            telemetry.addData("Ta",llresult.getTa());
        }

        /*
         *
         * ================
         *   MECANUM DRIVING INPUTS
         * ================
         *
         */

        double axial;
        double lateral;
        double yaw;
        double max;

        axial = -gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = -gamepad1.right_stick_x;

        /*
         *
         * ================
         *   SHOOTING ALIGNMENT
         * ================
         *
         */

        // if right trigger pressed, align the robot to the goal by turning the drivetrain
        if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.3){
            if (llresult.isValid()) {

                error = goalX - llresult.getTx();

                if (Math.abs(error) < angleTolerance) {
                    yaw = 0;

                } else {

                    telemetry.addData("Auto aligning","True");

                    double pTerm = error * kP;
                    currTime = getRuntime();
                    double deltaTime = currTime - lastTime;
                    double dTerm = ((error - lastError) / deltaTime) * kD;
                    telemetry.addData("Yaw", pTerm+dTerm);
                    if (pTerm +dTerm < -0.4) {
                        yaw = -0.4;
                    } else if (pTerm + dTerm > 0.4) {
                        yaw = 0.4;
                    } else {
                        yaw = pTerm + dTerm;
                    }

                    lastError = error;
                    lastTime = currTime;
                }

                alignment.update(llresult, hardwareMap);
                telemetry.addData("Tx", llresult.getTx());
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastTime = getRuntime();
            lastError = 0;
        }

        /*
         *
         * ================
         *   MECANUM DRIVING AND ALIGNMENT OUTPUTS
         * ================
         *
         */

        frontLeftPower = axial + lateral + yaw;
        frontRightPower = (axial - lateral) - yaw;
        backLeftPower = (axial - lateral) + yaw;
        backRightPower = (axial + lateral) - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (max > 1) {
            frontLeftPower = frontLeftPower / max;
            frontRightPower = frontRightPower / max;
            backLeftPower = backLeftPower / max;
            backRightPower = backRightPower / max;
        }
        // Send calculated power to wheels.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        /*
         *
         * ================
         *   SHOOTING
         * ================
         *
         */

        // if button b pressed, shoot
        if (driver1.getButton(GamepadKeys.Button.B)) {
            shooter.shoot();
        };


        /*
        *
        * ================
        *   INTAKE MECHANISM
        * ================
        *
        */

        // if left trigger pressed, turn intake inwards
        // if left bumper pressed, outtake the artefact
        // otherwise, no power to the motor
        if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
            telemetry.addData("Intaking","True");
            intakeMotor.setPower(-1*driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            telemetry.addData("Outtaking","True");
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        };
    }

}
