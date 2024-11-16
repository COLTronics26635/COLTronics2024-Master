package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Bare Bones", group="Ftc26635")
public class bareBones extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        mainArm = hardwareMap.get(DcMotor.class, "mainArm");
        hand = hardwareMap.get(DcMotor.class, "hand");

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");
        intakeServo = hardwareMap.get(CRServo.class, "Intake");

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Telemetry
            telemetry.addData("Status", "Running");
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
            }

            //Specimen Grabber
            if (gamepad1.x) {
                grabSpecimen(true);
            } else if (gamepad1.y) {
                grabSpecimen(false);
            }
            //Arm
            moveArm(gamepad1.left_trigger, gamepad1.left_bumper);
            moveHand(gamepad1.right_trigger, gamepad1.right_bumper);
            //Movement
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
        frontLeftDrive.setPower(-joyStick);
        backLeftDrive.setPower(-joyStick);
    }

    public void turn(double joyStick){
        //Use joystick value to turn
        frontRightDrive.setPower(-joyStick);
        backRightDrive.setPower(-joyStick);
        frontLeftDrive.setPower(-joyStick);
        backLeftDrive.setPower(-joyStick);
    }

    public void moveArm(float trigger, boolean bumper) {
        double maximumPosition = 550;
        double minimumPosition = 0;
        if (bumper  && hand.getCurrentPosition() <= maximumPosition) {
            mainArm.setPower(1);
        } else if (-trigger == 1  && hand.getCurrentPosition() >= minimumPosition) {
            mainArm.setPower(-trigger);
        }
    }

    public void moveHand(float trigger, boolean bumper) {
        double maximumPosition = 550;
        double minimumPosition = 0;
        if (bumper && hand.getCurrentPosition() <= maximumPosition) {
            hand.setPower(1);
        } else if (-trigger == 1 && hand.getCurrentPosition() >= minimumPosition) {
            hand.setPower(-trigger);
        }
    }
    public void takeIn(double power) {
        intakeServo.setPower(power);
    }
    public void grabSpecimen(boolean open) {
        double openPosition = 1;
        double closedPosition = 0;
        if (open) {
            specimenGrabber.setPosition(openPosition);
        } else {
            specimenGrabber.setPosition(closedPosition);
        }

    }
}
