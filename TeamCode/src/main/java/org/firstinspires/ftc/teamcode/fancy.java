package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
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

    //Declare Senors
    ColorSensor color;
    TouchSensor touch;
    IMU imu;
    BNO055IMU imu2;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //Declare  joyStick values:
    double left;
    double right;
    boolean open;

    @Override
    public void runOpMode() {

        //Initializing Motors:
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backRight");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        mainArm = hardwareMap.get(DcMotor.class, "mainArm");
        hand = hardwareMap.get(DcMotor.class, "hand");

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");
        intakeServo = hardwareMap.get(CRServo.class, "Intake");

        //specimenGrabber.setPosition(1);
        open = false;

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

            left = -gamepad1.left_stick_x;
            right = gamepad1.right_stick_y;


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
            telemetry.addData("hand pos", hand.getCurrentPosition());
            telemetry.addData("arm pos", mainArm.getCurrentPosition());
            telemetry.addData("Specimen Grabber Pos", specimenGrabber.getPosition());
            telemetry.addLine();
            telemetry.addData("Temp (℉)", imu2.getTemperature());
            telemetry.update();

            //Intake
            if (gamepad1.a) {
                takeIn(1);
            } else if (gamepad1.b) {
                takeIn(-1);
            } else if(!gamepad1.a && !gamepad1.b){
                takeIn(0);
            }

            //Specimen Grabber
            if (gamepad1.x) {
                grabSpecimen();
            }

            //Arm Movement
            moveArm(gamepad1.left_trigger, gamepad1.left_bumper);
            moveHand(gamepad1.right_trigger, gamepad1.right_bumper);

            //Drive Train Movement
            move(right, left);
            /*
            if (left != 0) {
                turn(left);
            } else {
                moveForward(right);
            }
            */
        }
    }

    /*public void moveForward(double joyStick){
        //Use Joystick value to move
        frontRightDrive.setPower(joyStick);
        backRightDrive.setPower(joyStick);
        frontLeftDrive.setPower(joyStick);
        backLeftDrive.setPower(joyStick);
    }
    public void turn(double joyStick){
        //Use joystick value to turn
        frontRightDrive.setPower(-joyStick);
        backRightDrive.setPower(-joyStick);
        frontLeftDrive.setPower(joyStick);
        backLeftDrive.setPower(joyStick);
    }*/
    public void move(double vertical, double horizontal){
        //Use joystick values to move
        double left = 0;
        double right = 0;

        left += vertical;
        right += vertical;

        left -= horizontal;
        right += horizontal;

        if (left > 1) {
            left = 1;
        }
        if(right > 1) {
            right = 1;
        }

        frontLeftDrive.setPower(left);
        backLeftDrive.setPower(left);
        frontRightDrive.setPower(right);
        backRightDrive.setPower(right);
    }


    public void moveArm(float trigger, boolean bumper) {
        double maximumPosition = -2300;
        double minimumPosition = 74;
        if (bumper) {
            mainArm.setPower(-1);
        } else if (trigger > 0) {
            mainArm.setPower(trigger);
        } else {
            mainArm.setPower(0);
        }
    }

    public void moveHand(float trigger, boolean bumper) {
        double maximumPosition = 10;
        double minimumPosition = -335;
        if (bumper && hand.getCurrentPosition() < maximumPosition) {
            hand.setPower(1);
        } else if (trigger > 0 && hand.getCurrentPosition() > minimumPosition) {
            hand.setPower(-trigger);
        } else {
            hand.setPower(0);
        }
    }
    public void takeIn(double power) {
        intakeServo.setPower(power);
    }
    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.8);
            open = false;
        } else if (!open) {
            specimenGrabber.setPosition(0.2);
            open = true;
        }
        delay(200);
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu2.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu2.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu2.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("gravity", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
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
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}