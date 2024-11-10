package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "SampleAutonomous", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {

    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
        }

        public class GrabSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }
        public Action grabSample() {
            return new GrabSample();
        }

        public class PlaceSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        }
        public Action placeSample() {
            return new PlaceSample();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-10, -65, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action path = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(75))
                .lineToY(-37)
                .waitSeconds(1)
                .setTangent(Math.toRadians(187))
                .lineToXLinearHeading(-48, Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(240))
                .lineToYLinearHeading(-50, Math.toRadians(225))
                .waitSeconds(1)
                .setTangent(Math.toRadians(120))
                .lineToYLinearHeading(-40, Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(290))
                .lineToYLinearHeading(-50, Math.toRadians(235))
                .build();

        Actions.runBlocking(
                path
        );
    }
}