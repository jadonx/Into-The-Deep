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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;

@Config
@Autonomous(name = "LeftSideAutonomous", group = "Autonomous")
public class LeftSideAutonomous extends LinearOpMode {

    public class ArmSlidesClaw {
        private DcMotor arm, leftSlide, rightSlide;
        private Servo leftClaw, rightClaw, claw;

        private ElapsedTime timer = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

            leftClaw = hardwareMap.get(Servo.class, "leftServo");
            rightClaw = hardwareMap.get(Servo.class, "rightServo");
            claw = hardwareMap.get(Servo.class, "clawServo");

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            arm.setDirection(DcMotor.Direction.REVERSE);
            leftSlide.setDirection(DcMotor.Direction.FORWARD);
            rightSlide.setDirection(DcMotor.Direction.REVERSE);
        }

        public void moveArm(int targetArm, double power) {
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
        }

        public void moveSlides(int targetSlides, double power) {
            leftSlide.setTargetPosition(targetSlides);
            rightSlide.setTargetPosition(targetSlides);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlide.setPower(power);
            rightSlide.setPower(power);
        }

        public void wristDown() {
            leftClaw.setPosition(0.6);
            rightClaw.setPosition(0.4);
        }

        public void wristDown90() {
            leftClaw.setPosition(0.9);
            rightClaw.setPosition(0.55);
        }

        public void wristPlaceSample() {
            leftClaw.setPosition(0.362);
            rightClaw.setPosition(0.65);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class TelemetryArmSlide implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Arm ", arm.getCurrentPosition());
                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());

                telemetry.update();

                return true;
            }
        }
        public Action telemetryArmSlide() {
            return new TelemetryArmSlide();
        }

        public class PlaceSample implements Action {
            private boolean wristPlaceSample = false;
            private boolean resetWrist = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Move arm
                moveArm(1570, 1);

                // Once arm reached target, move slides
                if (armReachedTarget(1570, 10)) {
                    moveSlides(1950, 1);
                }

                // Once slide reached target, move wrist
                if (slidesReachedTarget(1950, 10) && !wristPlaceSample) {
                    wristPlaceSample = true;
                    timer.reset();
                    wristPlaceSample();
                }

                // Once wrist is moving and timer has reached seconds, open claw
                if (wristPlaceSample && timer.seconds() > 0.5 && !resetWrist) {
                    resetWrist = true;
                    timer.reset();
                    claw.setPosition(0);
                }

                // Once claw is opened and timer reached target, reset wrist
                if (resetWrist && timer.seconds() > 0.3) {
                    wristDown();
                    return false;
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
                telemetry.addData("wristPlaceSample ", wristPlaceSample);
                telemetry.addData("resetWrist ", resetWrist);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return true;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }

        public class GrabSample implements Action {
            private boolean closeClaw = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveSlides(1250, 1);

                if (slidesReachedTarget(1250, 10) && !closeClaw) {
                    closeClaw = true;
                    claw.setPosition(1);
                    timer.reset();
                }

                telemetry.addData("Left Slide ", leftSlide.getCurrentPosition());
                telemetry.addData("Right Slide ", rightSlide.getCurrentPosition());
                telemetry.addData("Close Claw ", closeClaw);
                telemetry.addData("Timer ", timer.seconds());

                telemetry.update();

                return !closeClaw || !(timer.seconds() > 0.5);
            }
        }
        public Action grabSample() { return new GrabSample();}

        public class ResetArm implements Action {
            private boolean resetArm = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!resetArm) {
                    arm.setTargetPosition(0);
                    resetArm = true;
                    timer.reset();

                    arm.setPower(-1);
                }

                if (resetArm && timer.seconds() > 1) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }

                return true;
            }
        }
        public Action resetArm() { return new ResetArm(); }

        public class ResetSlides implements Action {
            private boolean resetSlides = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!resetSlides) {
                    leftSlide.setTargetPosition(0);
                    rightSlide.setTargetPosition(0);
                    resetSlides = true;
                    timer.reset();

                    leftSlide.setPower(-1);
                    rightSlide.setPower(-1);
                }

                if (resetSlides && timer.seconds() > 1) {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }

                return true;
            }
        }
        public Action resetSlides() { return new ResetSlides(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-37, -62, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeSample = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(45)), Math.toRadians(180));

        TrajectoryActionBuilder grabSample1 = placeSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .lineToXLinearHeading(-48, Math.toRadians(90));

        TrajectoryActionBuilder placeSample1 = grabSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(225))
                .lineToXLinearHeading(-57, Math.toRadians(45));

        TrajectoryActionBuilder grabSample2 = placeSample1.endTrajectory().fresh()
                .setTangent(Math.toRadians(96))
                .lineToXLinearHeading(-58, Math.toRadians(90));

        TrajectoryActionBuilder placeSample2 = grabSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(276))
                .lineToXLinearHeading(-57, Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = placeSample2.endTrajectory().fresh()
                .setTangent(Math.toRadians(81))
                .lineToXLinearHeading(-52, Math.toRadians(180));

        TrajectoryActionBuilder placeSample3 = grabSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(261))
                .lineToXLinearHeading(-57, Math.toRadians(45));

        TrajectoryActionBuilder parkAtSubmersible = placeSample3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, -10, Math.toRadians(180)), Math.toRadians(0));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                placeSample.build(),
                                armslidesclaw.placeSample()
                        ),
                        new ParallelAction(
                                grabSample1.build(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.resetArm()
                                )
                        ),
                        armslidesclaw.grabSample(),
                        new ParallelAction(
                                placeSample1.build(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.placeSample()
                                )
                        ),
                        new ParallelAction(
                                grabSample2.build(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.resetArm()
                                )
                        ),
                        armslidesclaw.grabSample(),
                        new ParallelAction(
                                placeSample2.build(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.placeSample()
                                )
                        ),
                        new ParallelAction(
                                grabSample3.build(),
                                new SequentialAction(
                                        armslidesclaw.resetSlides(),
                                        armslidesclaw.resetArm()
                                )
                        ),
                        placeSample3.build(),
                        parkAtSubmersible.build()
                )
        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        placeSample.build(),
//                        armslidesclaw.placeSample(),
//                        new ParallelAction(
//                                grabSample1.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        armslidesclaw.grabSample(),
//                        new ParallelAction(
//                                placeSample1.build(),
//                                armslidesclaw.resetSlides()
//                        ),
//                        armslidesclaw.placeSample(),
//                        new ParallelAction(
//                                grabSample2.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        armslidesclaw.grabSample(),
//                        new ParallelAction(
//                                placeSample2.build(),
//                                armslidesclaw.resetSlides()
//                        ),
//                        armslidesclaw.placeSample(),
//                        new ParallelAction(
//                                grabSample3.build(),
//                                new SequentialAction(
//                                        armslidesclaw.resetSlides(),
//                                        armslidesclaw.resetArm()
//                                )
//                        ),
//                        placeSample3.build(),
//                        parkAtSubmersible.build()
//                )
//        );

//        Actions.runBlocking(
//                new SequentialAction(
//                        placeSample.build(),
//                        grabSample1.build(),
//                        placeSample1.build(),
//                        grabSample2.build(),
//                        placeSample2.build(),
//                        grabSample3.build(),
//                        placeSample3.build(),
//                        parkAtSubmersible.build()
//                )
//        );
    }
}