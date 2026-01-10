package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Shooter{

    DcMotorEx launchMotor;
    Servo leftServo;
    Servo rightServo;

    double restingPosition;
    double launchSpeed;

    Servo statusFlag;

    Telemetry telemetry;

    private ElapsedTime waitTimer = new ElapsedTime();


    public Shooter(DcMotorEx LaunchMotor, Servo leftServo, Servo rightServo, Telemetry telemetry, Servo statusFlag){
        launchMotor = LaunchMotor;
        this.telemetry = telemetry;
        this.statusFlag = statusFlag;

        this.leftServo = leftServo;
        this.rightServo = rightServo;

        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        restingPosition = 0;
        launchSpeed = 0.6;
    }

    public double getCurrentSpeed(){
        return launchMotor.getVelocity();
    }

    public double getResting(){
        return restingPosition;
    }

    public void setResting(double set){
        telemetry.addData("ShooterResting", set);
        telemetry.update();
        restingPosition = set;
    }

    public double getLaunchSpeed(){
        return launchSpeed;
    }

    public void setLaunchSpeed(double launchSpeed){
        this.launchSpeed = launchSpeed;
    }

    public void SpeedUp(){
        launchMotor.setVelocity(1150);
    }

    public void Unstuck(){
        launchMotor.setVelocity(-1150);;
    }

    public void Shoot(int shootWait){

        if(waitTimer.seconds() > 1){
            if(launchMotor.getVelocity() < 1100 || launchMotor.getVelocity() > 1250) {
                int i = shootWait / 8;
                while (i > 0) {
                    telemetry.addData("Shooter", i);
                    telemetry.update();
                    launchMotor.setPower(0.6);

                    leftServo.setPosition(0.25);
                    rightServo.setPosition(0.75);

                    i--;
                }
            statusFlag.setPosition(0);
            }

            leftServo.setPosition(0.5 - restingPosition);
            rightServo.setPosition(0.5 + restingPosition);
            waitTimer.reset();
        }
    }

    public void UnShoot(int shootWait){

        int i = shootWait/8;
        while(i > 0){
            telemetry.addData("Shooter", i);
            telemetry.update();
            launchMotor.setPower(-1);

            leftServo.setPosition(0.75);
            rightServo.setPosition(0.25);

            i--;
        }

        leftServo.setPosition(0.5 - restingPosition);
        rightServo.setPosition(0.5 + restingPosition);
    }

    public void stopShooting(){
        launchMotor.setPower(-0.1);
        launchMotor.setPower(0);
        leftServo.setPosition(0.5 - restingPosition);
        rightServo.setPosition(0.5 + restingPosition);
    }
}
