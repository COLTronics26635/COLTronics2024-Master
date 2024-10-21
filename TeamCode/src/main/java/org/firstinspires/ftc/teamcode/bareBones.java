package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Bare Bones", group = "Ftc26635")
public class bareBones extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            //Create Vars
            double left;
            double right;

            //Set Status
            telemetry.addData("Status", "Running");
            telemetry.update();

            //Get Joystick Values
            left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            //Set Motor Power To Joystick position
            frontLeftDrive.setPower(left);
            backLeftDrive.setPower(left);
            frontRightDrive.setPower(right);
            backRightDrive.setPower(right);
        }
    }
}
