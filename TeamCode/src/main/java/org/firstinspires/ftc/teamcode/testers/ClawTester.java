package org.firstinspires.ftc.teamcode.testers;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Claw Tester")
public class ClawTester extends OpMode {
    public Servo claw;

    @Override
    public void init(){
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop(){
        if (gamepad1.x) {
            claw.setPosition(0);
        } else if (gamepad1.b) {
            claw.setPosition(1);
        }
    }
}