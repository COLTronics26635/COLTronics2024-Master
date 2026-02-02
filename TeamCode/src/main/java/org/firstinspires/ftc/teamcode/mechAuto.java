package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Mech")
public class mechAuto extends LinearOpMode {
    //Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;
    @Override
    public void runOpMode() {

        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (getRuntime() < 4) {
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(-1);
                backRightDrive.setPower(1);
                backLeftDrive.setPower(-1);
            } else {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
                backLeftDrive.setPower(0);
            }
        }
    }
}
//back right
//front right
//back left
//front left