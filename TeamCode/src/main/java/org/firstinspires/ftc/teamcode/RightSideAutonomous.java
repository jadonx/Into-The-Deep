package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;

@Config
@Autonomous(name = "Right - Autonomous", group = "Autonomous")
public class RightSideAutonomous extends LinearOpMode {

    public class ArmSlidesClaw {
        private DcMotorEx arm, leftSlide, rightSlide;
        private CRServo leftClaw, rightClaw;
        private Servo claw;

        private ElapsedTime runtime = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

            leftClaw = hardwareMap.get(CRServo.class, "leftServo");
            rightClaw = hardwareMap.get(CRServo.class, "rightServo");
            claw = hardwareMap.get(Servo.class, "clawServo");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class PlaceSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(100);

                leftClaw.setPower(0);
                rightClaw.setPower(0);


                claw.setPosition(1.0);

                moveArm(1450);

                sleep(300);

                moveSlides(645);

                moveArm(1545);

                sleep(200);


                moveSlides(1100);


                claw.setPosition(0);

                while (claw.getPosition() > 0.1) {

                }

                moveSlides(0);


                sleep(200);


                moveArm(0);


                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(50);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                claw.setPosition(1);


                return false;
            }
        }
        public Action placeSpecimen() {
            return new PlaceSpecimen();
        }

        public class PickupSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                moveArm(400);

                moveSlides(320);

                leftClaw.setPower(1);
                rightClaw.setPower(-1);

                sleep(500);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                sleep(750);

                moveArm(150);

                claw.setPosition(1); //closing claw

                while (claw.getPosition() < 0.9) {

                }

                moveSlides(0);

                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(500);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                return false;
            }
        }
        public Action pickupSample() {
            return new PickupSample();
        }

        public class pickupSpecimen1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                leftClaw.setPower(1);
                rightClaw.setPower(-1);

                sleep(190);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                return false;
            }
        }
        public Action pickupSpecimen1() {
            return new pickupSpecimen1();
        }public class pickupSpecimen2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);

                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(150);

                leftClaw.setPower(0);
                rightClaw.setPower(0);


                return false;
            }
        }
        public Action pickupSpecimen2() {
            return new pickupSpecimen2();
        }

        public void moveArm(int targetArm) {
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                telemetry.addData("Current arm position ", arm.getCurrentPosition());
                telemetry.update();
            }
        }

        public void moveSlides(int targetSlides) {
            leftSlide.setTargetPosition(-targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);

            while ((Math.abs(leftSlide.getCurrentPosition() + targetSlides) > 20) && (Math.abs(rightSlide.getCurrentPosition() - targetSlides) > 20)) {
                telemetry.addData("Left slide pos ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide pos ", rightSlide.getCurrentPosition());
                telemetry.update();
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(9, -62, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(9, -62, Math.toRadians(0)))
                    .setTangent(Math.toRadians(113))
                    .lineToYLinearHeading(-32.5, Math.toRadians(270))
                    ;


            TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                    .setTangent(Math.toRadians(280))
                    .lineToYLinearHeading(-40, Math.toRadians(270))
                    //move back from the cage

                    .setTangent(Math.toRadians(0))
                    .lineToXLinearHeading(35, Math.toRadians(0))

                    .setTangent(Math.toRadians(90))
                    .lineToYLinearHeading(-20, Math.toRadians(0))

                    .setTangent(Math.toRadians(45))
                    .lineToYLinearHeading(-10,Math.toRadians(0))
                    // first push point

                    .setTangent(Math.toRadians(270))
                    .lineToYLinearHeading(-50, Math.toRadians(0))

                    .setTangent(Math.toRadians(90))
                    .lineToY(-10)

                    .setTangent(Math.toRadians(0))
                    .lineToX(57)

                    .setTangent(Math.toRadians(270))
                    .lineToYLinearHeading(-50, Math.toRadians(0))

                    .setTangent(Math.toRadians(180))
                    .lineToX(36)

                    .setTangent(Math.toRadians(227))
                    .lineToX(29)
                    ;

            TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                    .setTangent(Math.toRadians(0))
                    .lineToX(40)
                    ;

            TrajectoryActionBuilder path4 = path3.endTrajectory().fresh()
                    .setTangent(Math.toRadians(148))
                    .lineToYLinearHeading(-32.5, Math.toRadians(270))
                    ;

            TrajectoryActionBuilder path5 = path4.endTrajectory().fresh()
                    .setTangent(Math.toRadians(318))
                    .lineToYLinearHeading(-57.5, Math.toRadians(0))
                    ;

            TrajectoryActionBuilder path6 = path5.endTrajectory().fresh()
                    .setTangent(Math.toRadians(0))
                    .lineToX(40)
                    ;

            TrajectoryActionBuilder path7 = path6.endTrajectory().fresh()
                    .setTangent(Math.toRadians(146))
                    .lineToYLinearHeading(-32.5, Math.toRadians(270))
                    ;

            TrajectoryActionBuilder path8 = path7.endTrajectory().fresh()
                    .setTangent(Math.toRadians(315))
                    .lineToYLinearHeading(-57.5, Math.toRadians(0))
                    ;

            TrajectoryActionBuilder path9 = path8.endTrajectory().fresh()
                    .setTangent(Math.toRadians(0))
                    .lineToX(40)
                    ;

            TrajectoryActionBuilder path10 = path9.endTrajectory().fresh()
                    .setTangent(Math.toRadians(148))
                    .lineToYLinearHeading(-32.5, Math.toRadians(270))
                    ;



            Actions.runBlocking(
                    new SequentialAction(
                            path1.build(),
                            path2.build(),
                            path3.build(),// place
                            path4.build(),
                            path5.build(),
                            path6.build(),
                            path7.build(),
                            path8.build(),
                            path9.build(),
                            path10.build()

                    )
            );

        }


    }
}