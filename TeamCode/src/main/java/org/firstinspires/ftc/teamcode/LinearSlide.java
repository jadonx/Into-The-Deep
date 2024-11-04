package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.controller.PIDController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "LinearSlideTest")
@Config
public class LinearSlide extends OpMode {

    public DcMotor leftSlide;
    public DcMotor rightSlide;

    // FTC Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double rightStickY = gamepad2.right_stick_y;

        leftSlide.setPower(rightStickY);
        rightSlide.setPower(rightStickY);

        if (leftSlide.getCurrentPosition() < 15) {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        } else if (leftSlide.getCurrentPosition() > 2100) {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        } else if (leftSlide.getCurrentPosition() > 1800 || leftSlide.getCurrentPosition() < 50) {
            rightStickY *= 0.3;
        }

        packet.put("Left Slide ", leftSlide.getCurrentPosition());
        packet.put("Right Slide ", rightSlide.getCurrentPosition());

        dashboard.sendTelemetryPacket(packet);
    }
}
