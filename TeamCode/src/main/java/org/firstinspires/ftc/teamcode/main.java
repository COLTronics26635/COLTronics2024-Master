package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Robot: Tank Drive", group="26635")
public class main extends LinearOpMode {
    @Override public void runOpMode() {
        //color Sensor
        ColorSensor color = hardwareMap.get(ColorSensor.class, "colorV2");

        //IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //Motors
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            double left;
            double right;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine()
                .addData("\nColor:\nRed", color.red())
                .addData("Green", color.green())
                .addData("Blue", color.blue());
            telemetry.addLine()
                    .addData("\nIMU:\nYaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES))
                    .addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES))
                    .addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES))
                    .addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate)
                    .addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate)
                    .addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            /*
            telemetry.addData("Color-Red", color.red());
            telemetry.addData("Color-Green", color.green());
            telemetry.addData("Color-Blue", color.blue());
            telemetry.addData("Color-Alpha", color.alpha());
            telemetry.addData("Color-ARGB", color.argb());

            telemetry.addData("IMU-Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("IMU-Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("IMU-Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("IMU-Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("IMU-Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("IMU-Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
            */


            //Get Joystick Values
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            //Set Motor Power To Joystick position
            frontLeftDrive.setPower(left);
            backLeftDrive.setPower(left);
            frontRightDrive.setPower(right);
            backRightDrive.setPower(right);
        }
    }
}
