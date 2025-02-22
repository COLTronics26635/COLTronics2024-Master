package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="2 Drivers", group = "main")
public class driver2 extends LinearOpMode {

    //Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    DcMotor arm;
    DcMotor hand;
    DcMotor intake;

    //Declare Servos
    Servo specimenGrabber;


    boolean open;
    double armMin;
    double armMax;


    @Override
    public void runOpMode() {

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(DcMotor.class, "hand");
        intake = hardwareMap.get(DcMotor.class, "intake");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        open = false;
        armMin = arm.getCurrentPosition();
        armMax = armMin - 1932;

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.update();


            //Intake
            if (gamepad2.a) {
                takeIn(1);
            } else if (gamepad2.b) {
                takeIn(-1);
            } else if(!gamepad2.a && !gamepad2.b){
                takeIn(0);
            }

            //Specimen Grabber
            if (gamepad2.x) {
                grabSpecimen();
            }

            //Arm Movement
            moveArm(gamepad2.left_stick_y);
            moveHand(gamepad2.right_stick_y);

            //Drive Train Movement
            move(gamepad1.right_stick_y, gamepad1.left_stick_x);

        }
    }

    public void move(double vertical, double horizontal){
        //Use joystick values to move
        double left = Range.clip(-vertical - horizontal, -1, 1);
        double right = Range.clip(-vertical + horizontal, -1, 1);

        frontLeftDrive.setPower(left);
        backLeftDrive.setPower(left);
        frontRightDrive.setPower(right);
        backRightDrive.setPower(right);
    }


    public void moveArm(float y) {
        if (y < 0 && arm.getCurrentPosition() > armMax) {
            arm.setPower(y);
        } else if (y > 0 && arm.getCurrentPosition() < armMin) {
            arm.setPower(y);
        } else {
            arm.setPower(0);
        }
    }

    public void moveHand(float y) {
        hand.setPower(y);
    }
    public void takeIn(double power) {
        intake.setPower(-power);
    }
    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.8);
        } else {
            specimenGrabber.setPosition(0.65);
        }
        open = !open;
        delay(200);
    }
    public void delay(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}