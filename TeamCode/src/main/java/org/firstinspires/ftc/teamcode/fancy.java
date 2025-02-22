package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

@TeleOp(name="Fancy", group = "other")
public class fancy extends LinearOpMode {

    //Declare the motors:
    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor backRightDrive;
    DcMotor backLeftDrive;

    DcMotor arm;
    DcMotor hand;
    DcMotor intake;


    //Declare Servo
    Servo specimenGrabber;

    //Declare Sensors
    ColorSensor color;
    TouchSensor touch;
    IMU imu;
    BNO055IMU imu2;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    boolean open;
    double armMin;
    double armMax;

    @Override
    public void runOpMode() {

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(DcMotor.class, "hand");
        intake = hardwareMap.get(DcMotor.class, "intake");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        open = false;
        armMin = arm.getCurrentPosition();
        armMax = armMin - 1932;

        //sensors
        imu = hardwareMap.get(IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        color = hardwareMap.get(ColorSensor.class, "color");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU2";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu2.initialize(parameters);

        composeTelemetry();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        imu2.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("IMU 1:\nYaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.addLine();
            telemetry.addData("Color:\nRed", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addLine();
            telemetry.addData("Touch:\ntouch", touch.isPressed());
            telemetry.addLine();
            telemetry.addData("Temp (℉)", imu2.getTemperature());
            telemetry.update();

            //Intake
            if (gamepad2.a) {
                takeIn(1);
            } else if (gamepad2.b) {
                takeIn(-1);
            } else if(!gamepad2.a && !gamepad2.b){
                takeIn(0);
            }

            //Specimen Grabber
            if (gamepad2.x) {
                grabSpecimen();
            }

            //Arm Movement
            moveArm(gamepad2.left_stick_y);
            moveHand(gamepad2.right_stick_y);

            //Drive Train Movement
            move(gamepad1.right_stick_y, gamepad1.left_stick_x);
        }
    }

    public void move(double vertical, double horizontal){
        //Use joystick values to move
        double left = Range.clip(-vertical - horizontal, -1, 1);
        double right = Range.clip(-vertical + horizontal, -1, 1);

        frontLeftDrive.setPower(left);
        backLeftDrive.setPower(left);
        frontRightDrive.setPower(right);
        backRightDrive.setPower(right);
    }
    public void moveArm(float y) {
        if (y < 0 && arm.getCurrentPosition() > armMax) {
            arm.setPower(y);
        } else if (y > 0 && arm.getCurrentPosition() < armMin) {
            arm.setPower(y);
        } else {
            arm.setPower(0);
        }
    }
    public void moveHand(float y) { hand.setPower(y); }
    public void takeIn(double power) { intake.setPower(-power); }
    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.8);
        } else {
            specimenGrabber.setPosition(0.65);
        }
        open = !open;
        delay(200);
    }
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() -> {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu2.getGravity();
        });

        telemetry.addLine()
                .addData("status", () -> imu2.getSystemStatus().toShortString())
                .addData("calib", () -> imu2.getCalibrationStatus().toString());

        telemetry.addLine()
                .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

        telemetry.addLine()
                .addData("gravity", () -> gravity.toString())
                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(gravity.xAccel * gravity.xAccel + gravity.yAccel * gravity.yAccel + gravity.zAccel * gravity.zAccel)));
    }


    public void delay(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}