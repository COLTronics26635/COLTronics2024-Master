package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Fancy", group="Ftc26635")
public class fancy extends LinearOpMode {

    //Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    DcMotor mainArm;
    DcMotor hand;

    //Declare Servos
    CRServo intakeServo;
    Servo specimenGrabber;


    //Declare joyStick values:
    double left;
    double right;

    boolean open;

    @Override
    public void runOpMode() {

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        mainArm = hardwareMap.get(DcMotor.class, "mainArm");
        hand = hardwareMap.get(DcMotor.class, "hand");

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");
        intakeServo = hardwareMap.get(CRServo.class, "Intake");

        //specimenGrabber.setPosition(1);
        open = false;

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("hand pos", hand.getCurrentPosition());
            telemetry.addData("arm pos", mainArm.getCurrentPosition());
            telemetry.addData("Specimen Grabber Pos", specimenGrabber.getPosition());
            telemetry.update();


            //Get Joystick Values
            left = -gamepad1.left_stick_x;
            right = gamepad1.right_stick_y;

            //Controller Button Map:

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
            if(left != 0) {
                turn(left);
            } else {
                moveForward(right);
            }
        }
    }

    public void moveForward(double joyStick){
        //Use Joystick value to move
        frontRightDrive.setPower(joyStick);
        backRightDrive.setPower(joyStick);
        frontLeftDrive.setPower(joyStick);
        backLeftDrive.setPower(joyStick);
    }
    public void turn(double joyStick){
        //Use joystick value to turn
        frontRightDrive.setPower(-joyStick);
        backRightDrive.setPower(-joyStick);
        frontLeftDrive.setPower(joyStick);
        backLeftDrive.setPower(joyStick);
    }


    public void moveArm(float trigger, boolean bumper) {
        double maximumPosition = -2300;
        double minimumPosition = 74;
        if (bumper) {
            mainArm.setPower(-1);
        } else if (trigger > 0) {
            mainArm.setPower(trigger);
        } else {
            mainArm.setPower(0);
        }
    }

    public void moveHand(float trigger, boolean bumper) {
        double maximumPosition = 10;
        double minimumPosition = -335;
        if (bumper && hand.getCurrentPosition() < maximumPosition) {
            hand.setPower(1);
        } else if (trigger > 0 && hand.getCurrentPosition() > minimumPosition) {
            hand.setPower(-trigger);
        } else {
            hand.setPower(0);
        }
    }
    public void takeIn(double power) {
        intakeServo.setPower(power);
    }
    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.3);
            open = false;
        } else if (!open) {
            specimenGrabber.setPosition(0.4);
            open = true;
        }
        try {
            Thread.sleep(200);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
