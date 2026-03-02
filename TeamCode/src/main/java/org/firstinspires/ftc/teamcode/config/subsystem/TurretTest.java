package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretTest {
    private Servo testServo;
    private double KP = 0.01;
    private double KD = 0.000;
    private double goalx = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private double position = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hwMap) {
        testServo = hwMap.get(Servo.class, "servo_pos");
        testServo.setPosition(0.5);
    }

    public void setKP(double newKP) {
        KP = newKP;
    }

    public double getKP() {
        return KP;
    }

    public void setKD(double newKD) {
        KP = newKD;
    }

    public double getKD() {
        return KD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(LLResult llresult) {
        double deltaTime = timer.seconds();
        resetTimer();

        if (llresult == null || !(llresult.isValid())) {
            lastError = 0;
            return;
        }

        double error = goalx - llresult.getTx();
        double pTerm = error * KP;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error-lastError)/deltaTime)*KD;
        }

        position = testServo.getPosition();

        if (Math.abs(error)>angleTolerance){
            //position = position + pTerm + dTerm;
            position = (error+90)/180;
        }

        testServo.setPosition(position);

        lastError = error;


        testServo.setPosition(position);
    }
}
