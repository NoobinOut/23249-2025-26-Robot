package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Shooter{

    DcMotor launchMotor;
    Servo leftServo;
    Servo rightServo;

    Telemetry telemetry;


    public Shooter(DcMotor LaunchMotor, Servo leftServo, Servo rightServo, Telemetry telemetry){
        launchMotor = LaunchMotor;
        this.telemetry = telemetry;

        this.leftServo = leftServo;
        this.rightServo = rightServo;

        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

    public void Shoot(int shootWait){
        int i = shootWait;
        while(i > 0){
            telemetry.addData("Shooter", i);
            telemetry.update();
            launchMotor.setPower(0.6);

            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);

            i--;
        }
        launchMotor.setPower(0.6);
        leftServo.setPosition(0.25);
        rightServo.setPosition(0.75);
    }

    public void stopShooting(){
        launchMotor.setPower(-0.1);
        launchMotor.setPower(0);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }
}
