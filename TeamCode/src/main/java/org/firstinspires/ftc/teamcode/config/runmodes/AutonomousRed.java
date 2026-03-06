package org.firstinspires.ftc.teamcode.config.runmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystem.AlignmentSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.ShooterSubsystem;

@Autonomous
public class AutonomousRed extends LinearOpMode {

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
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private Servo shooterAngleServo;

    // ---------------- PD Controller -------------- //

    double kP = 0.02;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.2;
    double kD = 0.005;
    double currTime = 0;
    double lastTime = 0;


    double servoVal = 5;
    double shooterPower = 5;
    double shooterCurrentPower = 0;

    double lastPressed = getRuntime();

    boolean shooting = false;

    boolean prevB = false;
    boolean currB;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

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


        /* instantiate limelight */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //april tag change this!

        /* instantiate alignment subsystem */
        alignment = new AlignmentSubsystem();

        /* instantiate intake motor */
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        /* instantiate shooter motors and servo */
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterAngleServo = hardwareMap.get(Servo.class, "shooterServo");

        telemetry.addLine("initialised all mechanics");

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 3.0)) {{
            limelight.start();
            resetRuntime();
            currTime = getRuntime();
        }
            LLResult llresult = limelight.getLatestResult();
            if (llresult.isValid()) {
                telemetry.addData("Ta", llresult.getTa());
            }


            if (opModeIsActive() && (runtime.seconds() < 3.0)) {
                if (llresult.isValid()) {
                    error = goalX - llresult.getTx();
                    if (Math.abs(error) < angleTolerance) {
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);;

                    } else {
                        telemetry.addData("Auto aligning", "True");

                        double pTerm = error * kP;
                        currTime = getRuntime();
                        double deltaTime = currTime - lastTime;
                        double dTerm = ((error - lastError) / deltaTime) * kD;
                        telemetry.addData("Yaw", pTerm + dTerm);
                        if (pTerm + dTerm < -0.4) {
                            frontLeft.setPower(-0.4);
                            frontRight.setPower(0.4);
                            backLeft.setPower(-0.4);
                            backRight.setPower(0.4);
                        } else if (pTerm + dTerm > 0.4) {
                            frontLeft.setPower(0.4);
                            frontRight.setPower(-0.4);
                            backLeft.setPower(0.4);
                            backRight.setPower(-0.4);
                        } else {
                            frontLeft.setPower(pTerm + dTerm);
                            frontRight.setPower(-(pTerm + dTerm));
                            backLeft.setPower(pTerm + dTerm);
                            backRight.setPower(-(pTerm + dTerm));
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
            }}

        runtime.reset();
        if (opModeIsActive() && (runtime.seconds() < 2.0)){
            shooterAngleServo.setPosition(0.95);
            shooterMotor1.setPower(0.3);
            shooterMotor2.setPower(-0.3);
            runtime.wait(2000);
            intakeMotor.setPower(1);
            runtime.wait(400);
            intakeMotor.setPower(0);
            runtime.wait(500);
            intakeMotor.setPower(1);
            runtime.wait(400);
            intakeMotor.setPower(0);
        }
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        shooterAngleServo.setPosition(0);

        runtime.reset();
        frontLeft.setPower(0.7);
        frontRight.setPower(0.7);
        backLeft.setPower(0.7);
        backRight.setPower(0.7);
        runtime.wait(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);;


    }

}

