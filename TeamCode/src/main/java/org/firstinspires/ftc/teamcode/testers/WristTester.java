package org.firstinspires.ftc.teamcode.testers;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Wrist Tester")
public class WristTester extends OpMode {
    public Servo left, right, claw;
    public ElapsedTime runtime;

    public static double leftPos, rightPos;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){
        left = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        leftPos = 0;
        rightPos = 0;

        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }

    @Override
    public void loop(){
        if (-gamepad1.left_stick_y > 0.1) {
            leftPos += 0.002;
        } else if (-gamepad1.left_stick_y < -0.1) {
            leftPos -= 0.002;
        }

        if (-gamepad1.right_stick_y > 0.1) {
            rightPos += 0.002;
        } else if (-gamepad1.right_stick_y < -0.1) {
            rightPos -= 0.002;
        }

        if (leftPos < -1) {
            leftPos = -1;
        } else if (leftPos > 1) {
            leftPos = 1;
        }

        if (rightPos < -1) {
            rightPos = -1;
        } else if (rightPos > 1) {
            rightPos = 1;
        }

        if (gamepad1.a) {
            wristDown90();
        } else if (gamepad1.b) {
            wristDown();
        } else if (gamepad1.y) {
            wristPlaceSample();
        }

        left.setPosition(leftPos);
        right.setPosition(rightPos);

        telemetry.addData("Left pos ", leftPos);
        telemetry.addData("Right pos ", rightPos);
        telemetry.update();
    }

    public void wristDown() {
        leftPos = 0.6;
        rightPos = 0.4;
    }

    public void wristDown90() {
        leftPos = 0.9;
        rightPos = 0.55;
    }

    public void wristPlaceSample() {
        leftPos = 0.362;
        rightPos = 0.65;
    }
}