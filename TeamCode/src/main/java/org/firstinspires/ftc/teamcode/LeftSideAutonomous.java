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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        public void closeClaw() {
            claw.setPosition(1);
        }

        public void openClaw() {
            claw.setPosition(0);
        }

        public void wristDown() {
            leftClaw.setPosition(0.256);
            rightClaw.setPosition(0.636);
        }

        public void wristUp() {
            leftClaw.setPosition(0.612);
            rightClaw.setPosition(0.266);
        }

        public void wristPlaceSample() {
            leftClaw.setPosition(0.524);
            rightClaw.setPosition(0.35);
        }

        public boolean slidesReachedTarget(int targetSlides, int threshold) {
            return Math.abs(leftSlide.getCurrentPosition() - targetSlides) < threshold && Math.abs(rightSlide.getCurrentPosition() - targetSlides) < threshold;
        }

        public boolean armReachedTarget(int targetArm, int threshold) {
            return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
        }

        public class PlaceSpecimen implements Action {
            private boolean timerStarted = false;
            private boolean runSlides = false;

            private int targetArm = 1600;
            private int targetSlides = 870;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wristUp();
                closeClaw();

                moveArm(targetArm, 1);

                if (armReachedTarget(targetArm, 20)) {
                    moveSlides(targetSlides, 1);
                    telemetry.addData("moving slides", null);
                }

                if (slidesReachedTarget(targetSlides, 20) && !timerStarted) {
                    timer.reset();
                    timerStarted = true;
                }

                if (timerStarted && timer.seconds() > 1) {
                    runSlides = true;
                }

                if (runSlides) {
                    moveSlides(950, 1);
                }

//                telemetry.addData("Arm ", arm.getCurrentPosition());
//                telemetry.addData("Arm target ", arm.getTargetPosition());
//                telemetry.addData("Left slide ", leftSlide.getCurrentPosition());
//                telemetry.addData("Right slide ", rightSlide.getCurrentPosition());
//                telemetry.addData("Left slide target ", leftSlide.getTargetPosition());
//                telemetry.addData("Right slide target ", rightSlide.getTargetPosition());
//                telemetry.addData("Timer ", timer.seconds());
//                telemetry.addData("Timer stared ", timerStarted);
//                telemetry.addData("Run loop ", runLoop);
//                telemetry.update();

                return !slidesReachedTarget(1100, 20);
            }
        }
        public Action placeSpecimen() {
            return new PlaceSpecimen();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                closeClaw();

                timer.reset();

                return timer.seconds() < 5;
            }
        }
        public Action actionCloseClaw() {
            return new CloseClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-14, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        TrajectoryActionBuilder placeSpecimenPath = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .setTangent(Math.toRadians(70))
                .lineToY(-31)
                .waitSeconds(1); // Place specimen

        TrajectoryActionBuilder grabSample1Path = placeSpecimenPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-40)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
                .setTangent(Math.toRadians((90)))
                .lineToY(-33)
                .waitSeconds(1); // Grab sample #1

        TrajectoryActionBuilder placeSample1Path = grabSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(250))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .lineToYLinearHeading(-40, Math.toRadians(90)) // Move to sample #2
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(1); // Grab sample #2

        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-53, Math.toRadians(45)) // Place sample #2
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample3Path = placeSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
                .setTangent(Math.toRadians(180))
                .lineToX(-61) // Grab sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample3Path = grabSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder parkAtSubmersiblePath = placeSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .lineToYLinearHeading(-10, Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .lineToX(-23); // Park at submersible


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                placeSpecimenPath.build(),
                                armslidesclaw.placeSpecimen()
                        ),
                        grabSample1Path.build(),
                        placeSample1Path.build(),
                        grabSample2Path.build(),
                        placeSample2Path.build(),
                        grabSample3Path.build(),
                        placeSample3Path.build(),
                        parkAtSubmersiblePath.build()
                )
        );


//        Actions.runBlocking(
//                new SequentialAction(
//                        armslidesclaw.actionCloseClaw()
//                )
//        );
    }
}