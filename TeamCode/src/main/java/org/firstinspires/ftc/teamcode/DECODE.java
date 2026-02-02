package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DECODE")
public class DECODE extends LinearOpMode {

    //Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    DcMotor intake;
    DcMotor flyWheel;

    DcMotor flyWheel2;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeft");
        backLeftDrive = hardwareMap.dcMotor.get("backLeft");
        frontRightDrive = hardwareMap.dcMotor.get("frontRight");
        backRightDrive = hardwareMap.dcMotor.get("backRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheel2 = hardwareMap.get(DcMotor.class, "flyWheel2");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.update();

            //Drive Train Movement
            move();

            if (gamepad2.x) {
                intake.setPower(1);
            } else if (gamepad2.y) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            flyWheel.setPower(-gamepad2.right_trigger);
            flyWheel2.setPower(-gamepad2.right_trigger);

        }
    }

    public void move() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }
}