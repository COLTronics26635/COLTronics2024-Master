package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="BNO055IMU Sensor", group="Test")
public class imu9 extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    AngularVelocity velocity;
    MagneticFlux magnet;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            velocity = imu.getAngularVelocity();
            magnet = imu.getMagneticFieldStrength();

            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Heading (Z)", angles.firstAngle);
            telemetry.addData("Roll (Y)", angles.secondAngle);
            telemetry.addData("Pitch (X)", angles.thirdAngle);
            telemetry.addLine();
            telemetry.addData("Gyro X (°/s)", velocity.xRotationRate);
            telemetry.addData("Gyro Y (°/s)", velocity.yRotationRate);
            telemetry.addData("Gyro Z (°/s)", velocity.zRotationRate);
            telemetry.addLine();
            telemetry.addData("Magnetometer X (µT)", magnet.x);
            telemetry.addData("Magnetometer Y (µT)", magnet.y);
            telemetry.addData("Magnetometer Z (µT)", magnet.z);
            telemetry.update();
        }
    }
}
