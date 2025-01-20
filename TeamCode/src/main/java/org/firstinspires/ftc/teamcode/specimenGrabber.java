package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Gripper", group="Test")
public class specimenGrabber extends LinearOpMode {

    Servo specimenGrabber;

    boolean open;

    @Override
    public void runOpMode() {

        specimenGrabber = hardwareMap.get(Servo.class, "specimenGrabber");

        specimenGrabber.setPosition(1);
        open = true;

        //Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();
            telemetry.addData("open", open);
            telemetry.addData("Specimen Grabber Pos", specimenGrabber.getPosition());
            telemetry.addData("Button", gamepad1.x);
            telemetry.update();

            //Specimen Grabber
            if (gamepad1.x) {
                grabSpecimen();
            }
        }
    }

    public void grabSpecimen() {
        if (open) {
            specimenGrabber.setPosition(0.3);
            open = false;
        } else if (!open) {
            specimenGrabber.setPosition(0.4);
            open = true;
        }
        try {
            Thread.sleep(200);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

    }
}
