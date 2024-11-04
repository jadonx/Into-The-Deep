package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "SampleAutonomous", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {
    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {

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

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}