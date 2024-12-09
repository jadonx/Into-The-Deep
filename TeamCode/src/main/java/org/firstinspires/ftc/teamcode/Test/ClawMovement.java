package org.firstinspires.ftc.teamcode.Test;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Claw Mover Tester")
public class ClawMovement extends OpMode {
    public Servo left, right, claw;
    public ElapsedTime runtime;

    @Override
    public void init(){
        left = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop(){
        if(gamepad1.a) {
            left.setPosition(1);
        }  else if (gamepad1.y) {
            left.setPosition(-1);
        } else if (gamepad1.x) {
            right.setPosition(1);
        } else if (gamepad1.b) {
            right.setPosition(-1);
        }

        telemetry.update();
    }
}