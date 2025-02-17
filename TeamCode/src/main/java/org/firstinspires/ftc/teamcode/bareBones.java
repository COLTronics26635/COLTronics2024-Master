package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="bareBones")
public class bareBones extends LinearOpMode {

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


    //Declare joyStick values:
    double left;
    double right;

    boolean open;

    double armMin;
    double armMax;
    double handMin;
    double handMax;


    @Override
    public void runOpMode() {

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(DcMotor.class, "hand");
        intake = hardwareMap.get(DcMotor.class, "intake");

        hand.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        open = false;

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMin = arm.getCurrentPosition();
        armMax = armMin - 2639;
        handMin = 310;
        handMax = -12;

        waitForStart();

        while (opModeIsActive()) {
            //Get Joystick Values
            left = -gamepad1.left_stick_x;
            right = gamepad1.right_stick_y;


            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Hand", hand.getCurrentPosition());
            telemetry.addData("Min", handMin);
            telemetry.addData("Max", handMax);
            telemetry.addData("left", left);
            telemetry.addData("right", right);
            telemetry.update();


            //Intake
            if (gamepad1.a) {
                takeIn(1);
            } else if (gamepad1.b) {
                takeIn(-1);
            } else if(!gamepad1.a && !gamepad1.b){
                takeIn(0);
            }

            //Specimen Grabber
            if (gamepad1.x) {
                grabSpecimen();
            }

            //Arm Movement
            moveArm(gamepad1.left_trigger, gamepad1.left_bumper);
            moveHand(gamepad1.right_trigger, gamepad1.right_bumper);

            //Drive Train Movement
            move(right, -left);

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


    public void moveArm(float trigger, boolean bumper) {
        if (bumper && arm.getCurrentPosition() > armMax) {
            arm.setPower(-1);
        } else if (trigger > 0 && arm.getCurrentPosition() < armMin) {
            arm.setPower(trigger);
        } else {
            arm.setPower(0);
        }
    }

    public void moveHand(float trigger, boolean bumper) {
        if (bumper) {
            hand.setPower(-1);
        } else if (trigger > 0) {
            hand.setPower(trigger);
        } else {
            hand.setPower(0);
        }
    }
    public void takeIn(double power) {
        intake.setPower(-power);
    }
    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.8);
            open = false;
        } else if (!open) {
            specimenGrabber.setPosition(0.65);
            open = true;
        }
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