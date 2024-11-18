package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

    boolean open = false;

    @Override
    public void runOpMode() {
        //color Sensor
        ColorSensor color = hardwareMap.get(ColorSensor.class, "colorV2");
        //Voltage Sensor
        VoltageSensor Voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        mainArm = hardwareMap.get(DcMotor.class, "mainArm");
        hand = hardwareMap.get(DcMotor.class, "hand");

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Touch sensor
        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");

        //IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //Servos
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");
        intakeServo = hardwareMap.get(CRServo.class, "Intake");



        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Touch:\nPressed?", touch.isPressed());
            telemetry.addLine();
            telemetry.addData("Color:\nRed", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            telemetry.addData("ARGB", color.argb());
            telemetry.addData("Distance", ((DistanceSensor) color).getDistance(DistanceUnit.MM));
            telemetry.addLine();
            telemetry.addData("IMU:\nYaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.addLine();
            telemetry.addData("Voltage", Voltage.getVoltage());
            telemetry.addLine();
            telemetry.addData("\nhand pos", hand.getCurrentPosition());
            telemetry.addData("arm pos", mainArm.getCurrentPosition());
            telemetry.addData("open", open);
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
        double maximumPosition = 3442;
        double minimumPosition = 0;
        if (bumper && mainArm.getCurrentPosition() <= 3442) {
            mainArm.setPower(1);
        } else if (trigger == 1 && mainArm.getCurrentPosition() >= 0) {
            mainArm.setPower(-1);
        } else {
            mainArm.setPower(0);
        }
    }

    public void moveHand(float trigger, boolean bumper) {
        double maximumPosition = 4;
        double minimumPosition = -25;
        if (bumper && hand.getCurrentPosition() >= maximumPosition) {
            hand.setPower(-1);
        } else if (trigger == 1 && hand.getCurrentPosition() >= minimumPosition) {
            hand.setPower(1);
        } else {
            hand.setPower(0);
        }
    }
    public void takeIn(double power) {
        intakeServo.setPower(power);
    }
    public void grabSpecimen() {

        double openPosition = 1;
        double closedPosition = 0;
        if (open) {
            specimenGrabber.setPosition(closedPosition);
            open = false;
        } else if (!open) {
            specimenGrabber.setPosition(openPosition);
            open = true;
        }

    }
}