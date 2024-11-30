package org.firstinspires.ftc.teamcode.Test;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Claw Mover Tester")
public class ClawMovement extends OpMode {
    public CRServo left, right;
    public Servo claw;
    public ElapsedTime runtime;

    @Override
    public void init(){
        left = hardwareMap.get(CRServo.class, "leftServo");
        right = hardwareMap.get(CRServo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop(){
        if(gamepad1.a) {
            left.setPower(1.0);
            right.setPower(-1.0);
        }  else if (gamepad1.y) {
            left.setPower(-1.0);
            right.setPower(1.0);
        } else {
            left.setPower(0.0);
            right.setPower(0.0);
        }

        if (gamepad1.x) {
            claw.setPosition(1);
        } else if (gamepad1.b) {
            claw.setPosition(0);
        }
    }
}