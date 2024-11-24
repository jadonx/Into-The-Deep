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

                sleep(200);

                leftClaw.setPower(0);
                rightClaw.setPower(0);


                claw.setPosition(1.0);

                moveArm(1400);

                sleep(500);

                moveSlides(700);

                moveArm(1550);

                moveSlides(1100);


                claw.setPosition(0);

                while (claw.getPosition() > 0.1) {

                }

                moveSlides(0);


                sleep(500);


                moveArm(0);


                leftClaw.setPower(-1);
                rightClaw.setPower(1);

                sleep(400);

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

                sleep(600);

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
        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSlidesClaw armslidesclaw = new ArmSlidesClaw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(10, -62, Math.toRadians(180)))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(123))
                .lineToYLinearHeading(-25.5, Math.toRadians(270))

                ;
        // đi chưa tới chamber, khi quay về phải đợi vài giây


        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-38)
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(35, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                .setTangent(Math.toRadians(0))
                .lineToX(49.5)
                ;

        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-55)
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                ;

        TrajectoryActionBuilder path4 = path3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(-5)
                .setTangent(Math.toRadians(0))
                .lineToX(70)
                ;

        TrajectoryActionBuilder path5 = path4.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-50.5)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(30, Math.toRadians(0))
                ;

        TrajectoryActionBuilder path6 = path5.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .lineToYLinearHeading(-35, Math.toRadians(270))
                ;

        TrajectoryActionBuilder path7 = path6.endTrajectory().fresh()
                .setTangent(Math.toRadians(320))
                .lineToYLinearHeading(-57.5, Math.toRadians(0))
                ;

        TrajectoryActionBuilder path8 = path7.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .lineToYLinearHeading(-35, Math.toRadians(270))
                ;

        TrajectoryActionBuilder path9 = path8.endTrajectory().fresh()
                .setTangent(Math.toRadians(320))
                .lineToYLinearHeading(-57.5, Math.toRadians(0))
                ;

        TrajectoryActionBuilder path10 = path9.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .lineToYLinearHeading(-35, Math.toRadians(270))
                ;




        Actions.runBlocking(
                new SequentialAction(
                        path1.build(), //Place specimen
                        armslidesclaw.placeSpecimen()  // place

                )
        );

//        TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(-10, -60, Math.toRadians(270)))
//                .setTangent(Math.toRadians(75))
//                .lineToY(-35);
//
//        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()
//                .setTangent(Math.toRadians(270))
//                .lineToY(-42)
//                .setTangent(Math.toRadians(180))
//                .lineToXLinearHeading(-56, Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
//                .lineToY(-39);
//
//        TrajectoryActionBuilder path3 = path2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(275))
//                .lineToYLinearHeading(-70, Math.toRadians(45));
//
//
//
//        Actions.runBlocking(
//                 new SequentialAction(
//                         path1.build(),
//                         armslidesclaw.placeSpecimen(),
//                         path2.build(),
//                         armslidesclaw.pickupSample(),
//                         path3.build(),
//                         armslidesclaw.placeSample()
//                 )
//        );

    }
}