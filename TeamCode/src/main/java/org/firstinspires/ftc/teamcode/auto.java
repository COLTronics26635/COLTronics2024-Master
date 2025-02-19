package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Move")
public class auto extends LinearOpMode {
    //    Declare the motors:
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

        waitForStart();

        moveDistance(100);
        while (opModeIsActive()) {

        }

    }
    public void moveDistance(double mm) {
        double rightDistance = (backRightDrive.getCurrentPosition() + (mm * 1.6));
        double leftDistance = (backLeftDrive.getCurrentPosition() + (mm * 1.6));

        backRightDrive.setTargetPosition((int) rightDistance);
        backLeftDrive.setTargetPosition((int) leftDistance);
    }
}
