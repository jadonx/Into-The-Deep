package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LinearSlidePIDF")
@Config
public class LinearSlidePIDF extends OpMode {
    private DcMotorEx leftSlide, rightSlide;
    private ElapsedTime timer = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    // PID Control
    public static double Kp = 0.0;
    public static double Kd = 0.0;
    public static double Kg = 0.0;

    public static int target = 0;
    public int lastError = 0;

    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        int currentPos = leftSlide.getCurrentPosition();

        int error = target - currentPos;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // PIDF calculation
        double output = (Kp * error) + (Kd * derivative) + Kg;

        leftSlide.setPower(output);
        rightSlide.setPower(output);

        lastError = error;

        timer.reset();

        // FTC Dashboard
        packet.put("Pos ", currentPos);
        packet.put("Target ", target);
        packet.put("Error ", error);
        dashboard.sendTelemetryPacket(packet);
    }
}
