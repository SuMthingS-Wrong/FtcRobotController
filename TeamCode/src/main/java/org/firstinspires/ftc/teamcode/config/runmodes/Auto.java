package org.firstinspires.ftc.teamcode.config.runmodes;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystem.*;

@Autonomous
public class Auto extends OpMode{
    ElapsedTime autoTimer = new ElapsedTime();
    int AUTO_TOTAL = 30;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        // Start Position ==> End Position
        
    }
    double PARK_BUFFER = 1.5; // (time it takes to park) + safety (we can experiment to find a better number)
    double timeRemaining = AUTO_TOTAL - autoTimer.seconds();
    enum Alliance {RED, BLUE};
    enum StartSide {FRONT, BACK};
    enum State {START, TURN90CCW, DRIVETOARTEFACT, INTAKE, TURN90CW, MOVETOSHOOTPOS, SHOOT, STOP};
    State state = State.START;
    Alliance alliance = Alliance.RED;
    StartSide startSide = StartSide.BACK;

    private MecanumDrive mecanum;
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor intakeMotor;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;

    @Override
    public void start() {
        autoTimer.reset();
        state = State.START;
    }

    @Override
    public void init() {
        /* instantiate motors */
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        mecanum = new MecanumDrive(
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435)
        );

        /* instantiate limelight */
        //limelight = hardwareMap.get(Limelight3A.class, "limelight"); //change this to device name
        //limelight.pipelineSwitch(0); //april tag change this!
        imu = hardwareMap.get(IMU.class, "imu");

    }

    @Override
    public void loop() {
        telemetry.addData("Current State", state);
        // Prevents penalties from robot running after autonomous timer
        if (timeRemaining <= PARK_BUFFER) {
            state = State.STOP;
        }
        switch (state){
            case START:
                // align to april tag
<<<<<<< Updated upstream
                ElapsedTime firstShootingTime = new ElapsedTime();
                if (firstShootingTime.seconds()<2){ //change time and refine
                    shooter.shoot();
                }
=======
                shooter.shoot();
>>>>>>> Stashed changes
                if (alliance == Alliance.RED){
                    state = State.TURN90CW;
                } else if (alliance == Alliance.BLUE){
                    state = State.TURN90CCW;
                }

            case TURN90CW:
                ElapsedTime turningTime = new ElapsedTime();
            case DRIVETOARTEFACT:
                ElapsedTime drivingTime = new ElapsedTime();
<<<<<<< Updated upstream
                if (drivingTime.seconds() < 2 ){ // change time and arguments
                    mecanum.driveRobotCentric(0,0,0);// change the vals with experimentation
=======
                if (drivingTime.seconds() < 2 ){
                mecanum.driveRobotCentric(0,0,0);
>>>>>>> Stashed changes
                }
                // drive at a speed for a set time
            case INTAKE:
                // Rotate Robot approx(90 deg anticlockwise) and drive forward
                ElapsedTime intakeTime = new ElapsedTime();
<<<<<<< Updated upstream
                if (intakeTime.seconds()<2){ //change time after
=======
                if (intakeTime.seconds()<2){
>>>>>>> Stashed changes
                    intakeMotor.setPower(-1.2);
                }
                intakeMotor.setPower(-1.2);
            case TURN90CCW:
                ElapsedTime rotatingTime = new ElapsedTime();
                if (rotatingTime.seconds()<2){ //change time and refine
<<<<<<< Updated upstream
                    mecanum.driveRobotCentric(0,0,0);// change the vals with experimentation
=======
                    shooter.shoot();
>>>>>>> Stashed changes
                }
                // turn 90 degrees ccw
            case MOVETOSHOOTPOS:
                ElapsedTime movingTime = new ElapsedTime();
                if (movingTime.seconds()<2){ //change time and refine
<<<<<<< Updated upstream
                    mecanum.driveRobotCentric(0,0,0);// change the vals with experimentation
=======
                    shooter.shoot();
>>>>>>> Stashed changes
                }
                // go back to shoot pos
                // align to april tag
            case SHOOT:
                ElapsedTime shootingTime = new ElapsedTime();
                if (shootingTime.seconds()<2){ //change time and refine
                    shooter.shoot();
                }

                //shoot
            case STOP:
                // stop everything
                intakeMotor.setPower(0);
                mecanum.driveRobotCentric(0,0,0);

        };
    }
}
