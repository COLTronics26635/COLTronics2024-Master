
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="George")
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
        //Move Forward for 4.5 feet
        while(opModeIsActive()){
            if(getRuntime() < 10){
                frontRightDrive.setPower(0.1);
                backRightDrive.setPower(0.1);
                frontLeftDrive.setPower(-0.1);
                backLeftDrive.setPower(-0.1);
            }
            else{
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
            }

        }
    }
}
