package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SampleDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Final TeleOp")
public class MergeCode extends OpMode {

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    int armPos;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickY, leftStickX, rightStickX;

    // PID for Field Centric
    public static double Kp = 0.5;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // FTC Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();



    public DcMotorEx arm;
    public DcMotorEx leftSlide, rightSlide;

    public long previousTime = 0;
    public double ePrevious = 0;
    public double eIntegral = 0;

    public static int target = 0;

    // PID for Linear Slide
    public static double p = 0.03, i = 0.3, d = 0.0002;

    //PID for arm
    public static double kp = 0.05, ki = 0.0, kd = 0.0;
    public static double kf = 0.1;

    public static int targetArm = 0;
    public static double armPower = 0;
    public static int slidePosition = 0;
    public static double servoPosition = 0;
    public static double clawPosition = 0;
    public static boolean slidesExtended = false;
    private final double ticks_in_degree = 700 / 180.0;







    //adaptiveClaw camera = new adaptiveClaw();

    Point center;
    double angle;
    double lat;
    double lon;



    public CRServo left, right;

    public Servo claw;
    public ElapsedTime runtime;
    //public double armPower;
    public double wristPower;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public double power;


    SampleDetection pipeline = new SampleDetection();


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);




        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide = hardwareMap.get(DcMotorEx .class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //controller = new PIDController(kp, ki, kd);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());






        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


        left = hardwareMap.get(CRServo.class, "leftServo");
        right = hardwareMap.get(CRServo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        runtime = new ElapsedTime();

        targetArm = 0;
        armPower = 0;
        slidePosition = 0;
        servoPosition = 0;
        clawPosition = 0;

        runtime.reset();
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        /*
        double jS = -gamepad2.left_stick_y;

        int leftEncoder = leftSlide.getCurrentPosition();
        int rightEncoder = rightSlide.getCurrentPosition();


        double power = pidController(leftEncoder, p,i,d);

        if(targetArm > 700){
            leftSlide.setPower(jS);
            rightSlide.setPower(jS);
        } else if(leftEncoder < 10000 && rightEncoder < 10000){
            leftSlide.setPower(jS);
            rightSlide.setPower(jS);
        }*/

/*
        if (gamepad2.left_trigger>0.2){
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);
            //targetArm = 0;
        }*/
//        moveMotor(power);



//        if (rightEncoder > leftEncoder) {
//            leftSlide.setPower(0);
//            rightSlide.setPower(0);
//        } else {
//        }

        /*
        packet.put("p", p);
        packet.put("i", i);
        packet.put("d", d);
        packet.put("kp", kp);
        packet.put("ki", ki);
        packet.put("kd", kd);
        packet.put("kf", kf);
        packet.put("Left Motor:", Math.abs(leftEncoder));
        packet.put("Right Motor:", Math.abs(rightEncoder));
        packet.put("Left-Right Motor:", Math.abs(leftEncoder) - Math.abs(rightEncoder));
        packet.put("Power", power);
        packet.put("Target", target);*/



        /*
        controller.setPID(kp, ki, kd);
        armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, targetArm);
        double ff = Math.cos(Math.toRadians(targetArm / ticks_in_degree)) * kf;

        double powerArm = pid + ff;


        if(gamepad2.a){
            targetArm = 0;
        }
        arm.setPower(powerArm);

        packet.put("Arm Power", powerArm);

        /*
        if(gamepad2.right_stick_y==0.0){
            powerArm = 0.05;
        } else {
            powerArm = gamepad2.right_stick_y;
        }

        arm.setPower(powerArm);

         */
        if(gamepad2.dpad_left || gamepad2.dpad_right){
            //linear slide and arm code

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);

            while(Math.abs(leftSlide.getCurrentPosition()-leftSlide.getTargetPosition())>20
                    && Math.abs(rightSlide.getCurrentPosition()-rightSlide.getTargetPosition())>20){

            }
            targetArm = 250;
            armPower = 0.3;
            slidePosition = 0;
            servoPosition = 1.0;


        } else if(gamepad2.dpad_up){
            //linear slide and arm code
            targetArm = 1600;
            armPower = 0.3;
            slidePosition = 2100;
            servoPosition = -1.0;
            clawPosition = 1.0;

        } else if(gamepad2.dpad_down){


            targetArm = 0;
            armPower = 0.5;
            slidePosition = 0;
            servoPosition = 1.0;
            clawPosition = 0.0;



        } else if(gamepad2.left_bumper){
            targetArm = 1700;
            armPower = 0.5;
            slidePosition = 0;
        }

        if(servoPosition==1.0){//claw moves for half a second in desired direction
            runtime.reset();
            while(runtime.milliseconds()<1500){
                left.setPower(servoPosition);
                right.setPower(-servoPosition);
            }
            servoPosition = 0;
        } else {
            left.setPower(0.0);
            right.setPower(0.0);
        }
        if(targetArm == 0){ //sets the slides down before moving the arm
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);

            while(leftSlide.getCurrentPosition()>5){

            }
        }


        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);

        while(arm.isBusy()){

            try {
                Thread.sleep(1000);  // 1000 milliseconds = 1 second
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        leftSlide.setTargetPosition(slidePosition);
        rightSlide.setTargetPosition(slidePosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        while(leftSlide.isBusy() || rightSlide.isBusy()){

        }
        claw.setPosition(clawPosition);

        if(servoPosition!=0){//claw moves for half a second in desired direction
            runtime.reset();
            while(runtime.milliseconds()<1500){
                left.setPower(servoPosition);
                right.setPower(-servoPosition);
            }
            servoPosition = 0;
        } else {
            left.setPower(0.0);
            right.setPower(0.0);
        }



        telemetry.addData("pos", armPos);
        telemetry.addData("targetArm", arm.getCurrentPosition());





        /*
        if (gamepad2.a) {
            if (angle < 50.0 || angle > 130.0) {
                left.setPower(0.3);
                right.setPower(0.3);
            } else if (angle < 80.0 || angle > 100.0){
                left.setPower(0.2);
                right.setPower(0.2);

            } else if (angle < 85.0 || angle > 95.0){
                left.setPower(0.15);
                right.setPower(0.15);

            } else if (angle < 88.0 || angle > 92.0){
                left.setPower(0.1);
                right.setPower(0.1);
            } else {
                left.setPower(0.0);
                right.setPower(0.0);
            }
        }

        if(gamepad2.b){
            if(angle<80){
                left.setPower(0.15);
                right.setPower(0.15);
            } else if(angle <88) {
                left.setPower(0.1);
                right.setPower(0.1);
            } else if (angle>100){
                left.setPower(-0.15);
                right.setPower(-0.15);
            } else if(angle>92){
                left.setPower(-0.1);
                right.setPower(-0.1);
            }
            else{
                left.setPower(0.0);
                right.setPower(0.0);
            }


        }*/
/*
        if(gamepad2.a){
            left.setPower(-1.0);
            right.setPower(1.0);
        } else if(gamepad2.b){
            left.setPower(1.0);
            right.setPower(-1.0);
        } else if(gamepad2.x){
            power = (90-angle)/90;
            if (Math.abs(power)<(0.06));

            left.setPower(power);
            right.setPower(power);

            double x = center.x;
            double y = center.y;

            leftStickX = -(x-320)/Math.sqrt((x*x)+(y*y));
            leftStickY = (y-180)/Math.sqrt((x*x)+(y*y));
            rightStickX = 0.0;

        } else{
            left.setPower(0.0);
            right.setPower(0.0);
        }

        if(gamepad2.y){
            claw.setPosition(0.0);
        }
        else{
            claw.setPosition(0.2);
        }*/



        if(gamepad2.x){ // moving to centralize the claw
            angle = pipeline.returnAngle();
            center = pipeline.returnCenter();

            double x = center.x;
            double y = center.y;
            if(Math.abs(x-320)>20 && Math.abs(y-180)>20){//setting the center
                moveRobotCentric(-(x-320)/Math.sqrt((x*x)+(y*y)), (y-180)/Math.sqrt((x*x)+(y*y)), 0);
            } else if(Math.abs(angle-90)>3){
                power = (90-angle)/90;
                left.setPower(power);
                right.setPower(power); //setting the angle
            } else {
                claw.setPosition(1.0); //open the claw
            }


        } else { // move normally using gamepad1
            robotOrientation = imu.getRobotYawPitchRollAngles();
            robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

            double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
            double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

            //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
            if (Math.abs(rightStickX) > 0.1) {
                targetYaw = robotYaw;
                lastError = 0;
            }

            if (gamepad1.x) {
                imu.resetYaw();
                targetYaw = 0;
                integralSum = 0;
                lastError = 0;
            }

            // PID Calculations
            double error = targetYaw - robotYaw;
            error = (error + Math.PI) % (2 * Math.PI) - Math.PI;

            // Compute PID Terms
            double derivative = (error - lastError) / timer.seconds();
            integralSum += error * timer.seconds();
            double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            double rotationPower = Math.abs(rightStickX) > 0.1 ? rightStickX : -correction;

            frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
            frontRight.setPower((rotY - rotX - rotationPower) / denominator);
            backLeft.setPower((rotY - rotX + rotationPower) / denominator);
            backRight.setPower((rotY + rotX - rotationPower) / denominator);

            lastError = error;
            timer.reset();

            packet.put("Target: ", Math.toDegrees(targetYaw));
            packet.put("Actual: ", Math.toDegrees(robotYaw));
            packet.put("Error: ", Math.toDegrees(error));
            packet.put("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));


            telemetry.addData("Angle: ", angle);
            telemetry.addData("Latitude: ", lat);
            telemetry.addData("Longitude: ", lon);
            telemetry.addData("Center: ", center);
            telemetry.addData("Xpower", leftStickX);

            telemetry.addData("Ypower", leftStickY);
        }



        // FTC Dashboard
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    public void moveRobotCentric(double x, double y, double rx){
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / d;
        double backLeftPower = (y - x + rx) / d;
        double frontRightPower = (y - x - rx) / d;
        double backRightPower = (y + x - rx) / d;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void holdSlidePosition(int targetPosition) {
        // Continuously apply power to hold the slides at the target position
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!gamepad2.dpad_down && !gamepad2.dpad_up) {  // Keep holding position until another button is pressed
            // Re-apply small corrective power if the slides are not at the target position
            if (leftSlide.getCurrentPosition() < targetPosition) {
                leftSlide.setPower(0.1);  // Adjust power as needed
            } else {
                leftSlide.setPower(0);
            }

            if (rightSlide.getCurrentPosition() < targetPosition) {
                rightSlide.setPower(0.1);  // Adjust power as needed
            } else {
                rightSlide.setPower(0);
            }
        }
    }

    public double pidController(int target, double p, double i, double d) {
        long currentTime = micros();
        double deltaT = ((double)(currentTime +- previousTime)) / 1.0e6;

        int e = rightSlide.getCurrentPosition() - target;
        double eDerivative = (e - ePrevious) / deltaT;
        eIntegral = eIntegral + e * deltaT;

        double u = (p * e) + (d * eDerivative) + (i * eIntegral);

        previousTime = currentTime;
        ePrevious = e;

        return u;
    }

    public long micros() {
        return System.nanoTime() / 1000;
    }

    public void moveMotor(double power) {
        rightSlide.setPower(power);
    }
}