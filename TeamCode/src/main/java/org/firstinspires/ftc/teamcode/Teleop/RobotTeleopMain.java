package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.MechanumDrive;

// Tells the drive controller to consider this a TeleOp and name it "TeleopMain"
@TeleOp(name = "TeleopMain")
// create this class such that it is an OpMode and can be referenced by other files
public class RobotTeleopMain extends OpMode {

    // establish the 4 drive motors
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;

    // Create the drive system
    MechanumDrive driveSystem;

    // all this is called when the init button is pressed
    @Override
    public void init() {

        // set the variables bellow to the DCMotors in the setup on the drive controller
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // init the drive with above DCMotors
        driveSystem = new MechanumDrive(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        // Friendly message
        telemetry.addData("Main","All Systems Online!");
    }

    // Called every "Frame" in a loop
    @Override
    public void loop(){
        // drive the robot using the player 1 gamepad
        driveSystem.drive(
                -gamepad1.left_stick_x, // Strafe X
                gamepad1.left_stick_y,  // Strafe Y
                -gamepad1.right_stick_x, // Rotation
                gamepad1.right_trigger); // Slow Mode
    }
}
