package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Fancy", group="Ftc26635")
public class fancy extends LinearOpMode {
    @Override
    public void runOpMode() {
        //color Sensor
        ColorSensor color = hardwareMap.get(ColorSensor.class, "colorV2");
        color.enableLed(false);

        //Touch sensor
        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");
        //IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
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

        while (opModeIsActive()) {
            double left;
            double right;
            Boolean colorLightOn = false;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Touch\nPressed?", touch.isPressed());
            telemetry.addData("Value", touch.getValue());
            telemetry.addData("Connection Info", touch.getConnectionInfo());
            telemetry.addData("Device Name", touch.getDeviceName());
            telemetry.addData("Version", touch.getVersion());
            telemetry.addData("Manufacture", touch.getManufacturer());
            telemetry.addLine();
            telemetry.addData("Color:\nRed", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            telemetry.addData("ARGB", color.argb());
            telemetry.addData("I2C Address", color.getI2cAddress());
            telemetry.addData("Connection Info", color.getConnectionInfo());
            telemetry.addData("Device Name", color.getDeviceName());
            telemetry.addData("Version", color.getVersion());
            telemetry.addData("Manufacture", color.getManufacturer());

            telemetry.addLine();
            telemetry.addData("IMU:\nYaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.addData("Version", imu.getVersion());
            telemetry.addData("Connection Info", imu.getConnectionInfo());
            //telemetry.addData("Robot Orientation", imu.getRobotOrientation(null, null, AngleUnit.DEGREES));
            telemetry.addData("Robot Orientation (Quaternion)", imu.getRobotOrientationAsQuaternion());
            telemetry.addData("Name", imu.getDeviceName());
            telemetry.addData("Manufacture", imu.getManufacturer());
            telemetry.update();

            //Get Joystick Values
            left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            if (gamepad1.a && !colorLightOn) {
                colorLightOn = true;
                color.enableLed(true);
            } else if (gamepad1.a && colorLightOn) {
                colorLightOn = false;
                color.enableLed(false);
            }

            //Set Motor Power To Joystick position
            frontLeftDrive.setPower(left);
            backLeftDrive.setPower(left);
            frontRightDrive.setPower(right);
            backRightDrive.setPower(right);
        }
    }
}