
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="George")
public class auto extends LinearOpMode {


//    Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    Servo specimenGrabber;

    @Override
    public void runOpMode() {

        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Time", getRuntime());
            telemetry.update();

            if(getRuntime() < 15.5){
                frontRightDrive.setPower(-0.1);
                backRightDrive.setPower(-0.1);
                frontLeftDrive.setPower(0.1);
                backLeftDrive.setPower(0.1);
            }
            if (getRuntime() > 15) {
                specimenGrabber.setPosition(0.65);
            }
            if (getRuntime() > 16 && getRuntime() < 17) {
                frontRightDrive.setPower(1);
                backRightDrive.setPower(1);
                frontLeftDrive.setPower(-1);
                backLeftDrive.setPower(-1);
            }
            if (getRuntime() > 17) {
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
            }


        }
    }

    public void moveDistance(double distance) {

    }
}
