package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Touch Sensor", group="Test")
public class touch extends LinearOpMode {

    TouchSensor touch;

    @Override
    public void runOpMode() {

        touch = hardwareMap.get(TouchSensor.class, "touch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("Touching", touch.isPressed());
            telemetry.update();
        }
    }
}
