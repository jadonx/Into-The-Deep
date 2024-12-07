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
@Autonomous(name = "LeftSideAutonomous", group = "Autonomous")
public class LeftSideAutonomous extends LinearOpMode {

    public class ArmSlidesClaw {
        private DcMotor arm, leftSlide, rightSlide;
        private CRServo leftClaw, rightClaw;
        private Servo claw;

        private ElapsedTime timer = new ElapsedTime();

        public ArmSlidesClaw(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

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

        TrajectoryActionBuilder placeSpecimenPath = drive.actionBuilder(new Pose2d(-14, -62, Math.toRadians(270)))
                .setTangent(Math.toRadians(70))
                .lineToY(-31)
                .waitSeconds(1); // Place specimen

        TrajectoryActionBuilder grabSample1Path = placeSpecimenPath.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .lineToY(-40)
                .setTangent(Math.toRadians(180))
                .lineToXLinearHeading(-47, Math.toRadians(90)) // Move to sample #1
                .setTangent(Math.toRadians((90)))
                .lineToY(-35)
                .waitSeconds(1); // Grab sample #1

        TrajectoryActionBuilder placeSample1Path = grabSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(250))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #1
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample2Path = placeSample1Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(115))
                .lineToYLinearHeading(-48, Math.toRadians(90)) // Move to sample #2
                .setTangent(Math.toRadians(90))
                .lineToY(-35) // Grab sample #2
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample2Path = grabSample2Path.endTrajectory().fresh()
                .lineToY(-48)
                .setTangent(Math.toRadians(295))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #2
                .waitSeconds(1);

        TrajectoryActionBuilder grabSample3Path = placeSample2Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-26, Math.toRadians(180)) // Move to sample #3
                .setTangent(Math.toRadians(180))
                .lineToX(-60) // Grab sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder placeSample3Path = grabSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .lineToYLinearHeading(-55, Math.toRadians(45)) // Place sample #3
                .waitSeconds(1);

        TrajectoryActionBuilder parkAtSubmersiblePath = placeSample3Path.endTrajectory().fresh()
                .setTangent(Math.toRadians(55))
                .lineToXLinearHeading(-23, Math.toRadians(180)); // Park at submersible


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            placeSpecimenPath.build(),
                            grabSample1Path.build(),
                            placeSample1Path.build(),
                            grabSample2Path.build(),
                            placeSample2Path.build(),
                            grabSample3Path.build(),
                            placeSample3Path.build(),
                            parkAtSubmersiblePath.build()
                    )
            );
        }



        /*
        Actions.runBlocking(
                new SequentialAction(
                        armslidesclaw.claw()
                )
        );
         */
    }
}