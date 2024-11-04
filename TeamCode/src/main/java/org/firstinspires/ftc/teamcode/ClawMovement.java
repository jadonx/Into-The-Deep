package org.firstinspires.ftc.teamcode;

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
        claw.scaleRange(0.4, 0.6);
        runtime = new ElapsedTime();

        runtime.reset();
    }

    @Override
    public void loop(){
        runtime.reset();

        double leftPower = gamepad2.left_stick_y;
        double rightPower = -gamepad2.right_stick_y;

        left.setPower(leftPower);
        right.setPower(rightPower);
        if(gamepad2.a)
            claw.setPosition(1.0);
        else
            claw.setPosition(0.0);

    }
}