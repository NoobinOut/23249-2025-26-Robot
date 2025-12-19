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

    final double speedMultiplier = 0.5;

    Shooter shooter;

    boolean lastB = false;
    boolean lastY = false;

    // all this is called when the init button is pressed
    @Override
    public void init() {

        // set the variables bellow to the DCMotors in the setup on the drive controller
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        // drive the robot using the player 1 gamepad
        drive(
                -gamepad1.left_stick_x,
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

        if(gamepad1.b && !lastB && (shooter.getCurrentSpeed() > 1075 && shooter.getCurrentSpeed() < 1155)){
            lastB = true;
            shooter.Shoot(75000);
        }
        else if(!gamepad1.b){
            lastB = false;
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
        telemetry.update();
    }

    synchronized public void drive(double xDir, double yDir) {

        double overallPower = Math.sqrt(Math.pow(xDir, 2) + Math.pow(yDir, 2));
        double maxPowerMultiplier = 1 / overallPower;

        double xSign = Math.signum(xDir);
        double ySign = Math.signum(yDir);


        if(xSign == 0){
            xSign = 1;
        }

        if(ySign == 0){
            ySign = 1;
        }

        boolean stickRight = xDir >= 0;
        // boolean stickUp = yDir >= 0;

        double rightPower;
        double leftPower;

        if(xSign * ySign < 0)
        {
            rightPower = ySign;
        }
        else
        {
            if(stickRight) // true
            {
                rightPower = ((yDir * maxPowerMultiplier) - 0.5) * 2;
            }
            else
            {
                rightPower = ((yDir * maxPowerMultiplier) + 0.5) * 2;
            }
        }

        if(xSign * ySign > 0)
        {
            leftPower = xSign;
        }

        else
        {
            if(stickRight)
            {
                leftPower = ((yDir * maxPowerMultiplier) + 0.5) * 2;
            }
            else
            {
                leftPower = ((yDir * maxPowerMultiplier) - 0.5) * 2;
            }
            if (gamepad1.cross) {
                leftPower = 2;


            }

        }


        motorLeft.setPower(leftPower * overallPower * speedMultiplier);
        motorRight.setPower(rightPower * overallPower * speedMultiplier * -1);


    }

}


