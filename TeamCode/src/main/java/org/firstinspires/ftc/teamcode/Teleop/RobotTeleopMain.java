package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

// Tells the drive controller to consider this a TeleOp and name it "TeleopMain"
@TeleOp(name = "TeleopMain")
// create this class such that it is an OpMode and can be referenced by other files
public class RobotTeleopMain extends OpMode {

    // establish the 2 drive motors
    DcMotor motorLeft;
    DcMotor motorRight;

    DcMotorEx launchMotor;
    Servo leftServo;
    Servo rightServo;
    Servo statusFlag;

    final double speedMultiplier = 1;

    Shooter shooter;
    boolean lastY = false;

    // all this is called when the init button is pressed
    @Override
    public void init() {

        // set the variables bellow to the DCMotors in the setup on the drive controller
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        statusFlag = hardwareMap.servo.get("status");


        launchMotor = hardwareMap.get(DcMotorEx.class,"motorLaunch");

        leftServo = hardwareMap.servo.get("servoLeft");
        rightServo = hardwareMap.servo.get("servoRight");

        shooter = new Shooter(launchMotor, leftServo, rightServo, telemetry);

        // Friendly message
        telemetry.addData("Main","All Systems Online!");
        telemetry.update();
    }

    // Called every "Frame" in a loop
    @Override
    public void loop(){

        if(shooter.getCurrentSpeed() > 1100 && shooter.getCurrentSpeed() < 1250){
            statusFlag.setPosition(1);
        }
        else{
            statusFlag.setPosition(0);
        }

        // drive the robot using the player 1 gamepad
        drive(
                -gamepad1.right_stick_x,
                gamepad1.left_stick_y);

        if(gamepad1.left_trigger > 0.25){
            shooter.Unstuck();
        }
        else if(gamepad1.right_trigger > 0.25){
            shooter.SpeedUp();
        }

        if(gamepad1.dpad_up){
            shooter.setResting(shooter.getResting() + 0.001);
        }
        else if(gamepad1.dpad_down){
            shooter.setResting(shooter.getResting() - 0.001);
        }

        if(gamepad1.b && (shooter.getCurrentSpeed() > 1100 && shooter.getCurrentSpeed() < 1250)){
            shooter.Shoot(75000);
        }

        if(gamepad1.y && !lastY){
            lastY = true;
            shooter.UnShoot(75000);
        }
        else if(!gamepad1.y){
            lastY = false;
        }

        if(!(gamepad1.left_trigger > 0.25) && !(gamepad1.right_trigger > 0.25) && !gamepad1.b && !gamepad1.y){
            shooter.stopShooting();
        }

        telemetry.addData("Shooter Speed", shooter.getCurrentSpeed());
        telemetry.addData("CAN SHOOT", (shooter.getCurrentSpeed() > 1100 && shooter.getCurrentSpeed() < 1250));
        telemetry.addData("Feed Wheel Angle", leftServo.getPosition());
        telemetry.update();
    }

    synchronized public void drive(double xDir, double yDir) {

        double maxPowerMultiplier = 0.75;
        double deadzone = 0.15;

        if(xDir > deadzone | xDir < -deadzone){
            motorLeft.setPower(xDir * speedMultiplier);
            motorRight.setPower(xDir * speedMultiplier);
        }
        else if(yDir > deadzone | yDir < -deadzone){
            motorLeft.setPower(yDir * speedMultiplier);
            motorRight.setPower(yDir * speedMultiplier * -1);
        }
        else{
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }

}


