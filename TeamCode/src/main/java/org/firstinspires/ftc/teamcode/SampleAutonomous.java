package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.ParallelAction;
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

@Config
@Autonomous(name = "SampleAutonomous", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {

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
            private int targetArm = 1560;
            private int targetSlides = 1100;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1.0);

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);

                while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                    telemetry.addData("Current arm position ", arm.getCurrentPosition());
                    telemetry.update();
                }

                sleep(1000);

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

                sleep(1000);

                claw.setPosition(0);

                targetSlides = 0;

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

                targetArm = 0;

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);

                while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                    telemetry.addData("Current arm position ", arm.getCurrentPosition());
                    telemetry.update();
                }

                return false;
            }
        }
        public Action placeSpecimen() {
            return new PlaceSpecimen();
        }

        public class PlaceSample implements Action {
            private int targetArm = 1650;
            private int targetSlides = 2100;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1.0);

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);

                while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                    telemetry.addData("Current arm position ", arm.getCurrentPosition());
                    telemetry.update();
                }

                sleep(1000);

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

                claw.setPosition(0);

                return false;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class PickupSample implements Action {
            private int targetArm = 400;
            private int targetSlides = 320;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);

                while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                    telemetry.addData("Current arm position ", arm.getCurrentPosition());
                    telemetry.update();
                }

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

                leftClaw.setPower(1);
                rightClaw.setPower(-1);

                sleep(600);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                sleep(750);

                targetArm = 150;

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);

                while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                    telemetry.addData("Current arm position ", arm.getCurrentPosition());
                    telemetry.update();
                }

                claw.setPosition(1); //closing claw

                targetSlides = 0;

                leftSlide.setTargetPosition(-targetSlides);
                rightSlide.setTargetPosition(targetSlides);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlide.setPower(0.14);
                rightSlide.setPower(0.14);

                while ((Math.abs(leftSlide.getCurrentPosition() + targetSlides) > 20) && (Math.abs(rightSlide.getCurrentPosition() - targetSlides) > 20)) {
                    telemetry.addData("Left slide pos ", leftSlide.getCurrentPosition());
                    telemetry.addData("Right slide pos ", rightSlide.getCurrentPosition());
                    telemetry.update();
                }

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

        public class Claw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                return false;
            }
        }
        public Action claw() {
            return new Claw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-10, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Action path = drive.actionBuilder(new Pose2d(-10, -60, Math.toRadians(270)))
                .setTangent(Math.toRadians(75))
                .lineToY(-37)
                .waitSeconds(3)
                .lineToY(-42)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-62, Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(240))
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(105))
                .lineToYLinearHeading(-42, Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(285))
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .build();

        TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(-10, -60, Math.toRadians(270)))
                .setTangent(Math.toRadians(75))
                .lineToY(-35);

        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-42)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-56, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(-39);

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .setTangent(Math.toRadians(275))
                .lineToYLinearHeading(-70, Math.toRadians(45));

        Actions.runBlocking(
                 new SequentialAction(
                         armslidesclaw.claw(),
                         path1.build(),
                         armslidesclaw.placeSpecimen(),
                         path2.build(),
                         armslidesclaw.pickupSample(),
                         path3.build(),
                         armslidesclaw.placeSample()
                 )
        );

    }
}