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

        private ElapsedTime timer = new ElapsedTime();

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
                claw.setPosition(1.0);

                moveArm(1560, 0.7);

                sleep(500);

                moveSlides(1050, 0.8);

                sleep(300);

                claw.setPosition(0);

                moveSlides(0, 0.8);

                moveArm(0, 1);

                return false;
            }
        }
        public Action placeSpecimen() {
            return new PlaceSpecimen();
        }

        public class PlaceSample implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1.0);

                moveArm(1740, 1);

                sleep(1000);

                moveSlides(2050, 1);

                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(330);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                claw.setPosition(0);

                timer.reset();

                while (timer.milliseconds() < 600) {

                }

                moveSlides(0, 1);

                moveArm(0, 0.6);

                return false;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class PickupSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);

                moveArm(350, 0.5);

                leftClaw.setPower(1);
                rightClaw.setPower(-1);

                sleep(600);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                sleep(850);

                moveArm(-50, 1);

                claw.setPosition(1); //closing claw

                timer.reset();

                while (timer.milliseconds() < 1000) {

                }

                moveArm(350, 0.7);

                return false;
            }
        }
        public Action pickupSample() {
            return new PickupSample();
        }

        public class Claw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);

                timer.reset();

                while (timer.milliseconds() < 1000) {

                }

                claw.setPosition(0);

                timer.reset();

                while (timer.milliseconds() < 1000) {

                }

                claw.setPosition(1);

                while (claw.getPosition() < 0.9) {
                    claw.setPosition(1);
                    telemetry.addData("Claw pos: ", claw.getPosition());
                    telemetry.update();
                }

                sleep(1000);

                leftClaw.setPower(1);
                rightClaw.setPower(-1);

                sleep(600);

                leftClaw.setPower(0);
                rightClaw.setPower(0);

                sleep(1000);

                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(600);

                return false;
            }
        }
        public Action claw() {
            return new Claw();
        }

        public void moveArm(int targetArm, double power) {
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);

            while (Math.abs(arm.getCurrentPosition() - targetArm) > 20) {
                telemetry.addData("Current arm position ", arm.getCurrentPosition());
                telemetry.update();
            }
        }

        public void moveSlides(int targetSlides, double power) {
            leftSlide.setTargetPosition(-targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(power);
            rightSlide.setPower(power);

            while ((Math.abs(leftSlide.getCurrentPosition() + targetSlides) > 20) && (Math.abs(rightSlide.getCurrentPosition() - targetSlides) > 20)) {
                telemetry.addData("Left slide pos ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide pos ", rightSlide.getCurrentPosition());
                telemetry.update();
            }
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
                .lineToXLinearHeading(-54, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(-35);

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToYLinearHeading(-62, Math.toRadians(45));

        TrajectoryActionBuilder path4 = path3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-45, Math.toRadians(90));

        TrajectoryActionBuilder path5 = path4.endTrajectory().fresh()
                .setTangent(Math.toRadians(285))
                .lineToYLinearHeading(-43, Math.toRadians(45));

        Actions.runBlocking(
                 new SequentialAction(
                         path1.build(),
                         armslidesclaw.placeSpecimen(),
                         path2.build(),
                         armslidesclaw.pickupSample(),
                         path3.build(),
                         armslidesclaw.placeSample(),
                         path4.build(),
                         armslidesclaw.pickupSample()
                 )
        );

        /*
        Actions.runBlocking(
                new SequentialAction(
                        armslidesclaw.claw()
                )
        );
         */
    }
}