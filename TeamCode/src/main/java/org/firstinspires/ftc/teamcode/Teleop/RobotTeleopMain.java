package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

// Tells the drive controller to consider this a TeleOp and name it "TeleopMain"
@TeleOp(name = "TeleopMain")
// create this class such that it is an OpMode and can be referenced by other files
public class RobotTeleopMain extends OpMode {

    // establish the 2 drive motors
    DcMotor motorLeft;
    DcMotor motorRight;

    DcMotor launchMotor;
    Servo leftServo;
    Servo rightServo;

    final double speedMultiplier = 0.5;

    Shooter shooter;

    boolean lastB = false;

    // all this is called when the init button is pressed
    @Override
    public void init() {

        // set the variables bellow to the DCMotors in the setup on the drive controller
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");


        launchMotor = hardwareMap.dcMotor.get("motorLaunch");

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

        if(gamepad1.b && !lastB){
            lastB = true;
            shooter.Shoot(75000);
        }
        else if(!gamepad1.b){
            lastB = false;
            shooter.stopShooting();
        }
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


