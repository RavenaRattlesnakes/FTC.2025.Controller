package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AutoColor")
public class Auto_Color extends LinearOpMode {

    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor MoonMotor = null;
    private DcMotor ScoopMotor = null;
    private DcMotor ArmMotor = null;
    private Servo LeftServo = null;
    private Servo RightServo = null;
    private Servo MiddleServo = null;
    private BHI260IMU imu = null;

    @Override
    public void runOpMode() {

        // Initialize hardware
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        MoonMotor = hardwareMap.get(DcMotor.class, "MoonMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ScoopMotor = hardwareMap.get(DcMotor.class, "ScoopMotor");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");
        MiddleServo = hardwareMap.get(Servo.class, "MiddleServo");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Initialize IMU parameters
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.resetYaw();


        imu.initialize(parameters);

        // Motor configuration
        MoonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MoonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        ScoopMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftServo.setPosition(1);
        RightServo.setPosition(0);
        MiddleServo.setPosition(0.9);


        resetEncoders();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example pathway

            ArmDown(100,0.7);
            UpSlide(800,0.7);
            Backward(1650,1.0);
            DownSlide(350,0.9);
            RightServo.setPosition(1);
            LeftServo.setPosition(0);
            DownSlide(450,0.7);
            Forward(110,1.0);
            StrafeLeft(1656,0.8);
            //SlideMove(1766,1546,1546,1766,0.8);
            Backward(1329,1.0);
            StrafeLeft(465,0.8);
            //SlideMove(864,1794,1794,864, 0.8);
            Forward(2302,1.0);
            Backward(521,1.0);
            TurnLeft(2325,0.8);
            OpenServos();
            sleep(750);
            Backward(940,0.5);
            sleep(500);
            CloseServos();
            Forward(900,1.0);
            TurnRight(2300,0.8);
            StrafeRight(2512,0.8);
            UpSlide(800,0.79);
            Backward(700,1.0);
            DownSlide(350,0.9);
            RightServo.setPosition(1);
            LeftServo.setPosition(0);
            DownSlide(450,0.79);
            ArmUp(100,1.0);
//            StrafeLeft(457,0.7);
//            Forward(2302,0.7);
//            Backward(2302,0.7);
//            StrafeLeft(400,0.7);
//            Forward(2302,0.7);
//
//





        }
    }

    private void stopMotors() {
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
    }


    private void Forward(int ticks, double speed) {
        moveRobot(ticks, ticks, ticks, ticks, speed);
    }

    private void Backward(int ticks, double speed) {
        moveRobot(-ticks, -ticks, -ticks, -ticks, speed);
    }

    private void StrafeRight(int ticks, double speed) {
        moveRobot(-ticks, ticks, ticks, -ticks, speed);
    }

    private void SlideMove(int tick1, int tick2, int tick3, int tick4, double speed) {
        moveRobot(tick1, tick2, tick3, tick4, speed);
    }

    private void StrafeLeft(int ticks, double speed) {
        moveRobot(ticks, -ticks, -ticks, ticks, speed);
    }


    private void moveRobot(int backLeftTicks, int backRightTicks, int frontLeftTicks, int frontRightTicks, double speed) {
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int MaxTick1 = Math.max(backLeftTicks, backRightTicks);
        int MaxTick2 = Math.max(MaxTick1, frontLeftTicks);
        int MaxTicks = Math.max(MaxTick2, frontRightTicks);
        double RFSpeed = speed * (frontRightTicks/MaxTicks);
        double LFSpeed = speed * (frontLeftTicks/MaxTicks);
        double RRSpeed = speed * (backRightTicks/MaxTicks);
        double LRSpeed = speed * (backLeftTicks/MaxTicks);

        BackLeftDrive.setTargetPosition(backLeftTicks);
        BackRightDrive.setTargetPosition(backRightTicks);
        FrontLeftDrive.setTargetPosition(frontLeftTicks);
        FrontRightDrive.setTargetPosition(frontRightTicks);

        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BackLeftDrive.setPower(speed);
        BackRightDrive.setPower(speed);
        FrontLeftDrive.setPower(speed);
        FrontRightDrive.setPower(speed);

        while (opModeIsActive() &&
                (BackLeftDrive.isBusy() || BackRightDrive.isBusy() ||
                        FrontLeftDrive.isBusy() || FrontRightDrive.isBusy())) {
            telemetry.update();
        }

        stopMotors();
    }

    private void resetEncoders() {
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ScoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ScoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MoonMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Turn the robot left by a specified angle
    private void TurnLeft(int ticks, double speed) {
        moveRobot(-ticks, ticks, -ticks, ticks, speed);
    }

    // Turn the robot right by a specified angle
    private void TurnRight(int ticks, double speed) {
        moveRobot(ticks, -ticks, ticks, -ticks, speed);

    }

    private void CloseServos() {
        LeftServo.setPosition(1);
        RightServo.setPosition(0);
    }

    private void OpenServos() {
        LeftServo.setPosition(0.45);
        RightServo.setPosition(0.45);
    }

    private void moveSlide (int MoonMotorTicks, double speed) {
        MoonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MoonMotor.setTargetPosition(MoonMotorTicks);

        MoonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MoonMotor.setPower(speed);


        while (!(!opModeIsActive() ||
                !MoonMotor.isBusy())) {
            telemetry.update();
        }
    }
    private void UpSlide (int ticks, double speed)  {
        moveSlide(-ticks,speed);
    }

    private void DownSlide (int ticks, double speed) {
        moveSlide(ticks, speed);
    }

    private void moveArm (int ArmMotorTicks, double speed) {
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmMotor.setTargetPosition(ArmMotorTicks);

        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmMotor.setPower(speed);

        while (!(!opModeIsActive() ||
                !ArmMotor.isBusy())) {
            telemetry.update();
        }
    }

    private void ArmDown (int ticks, double speed) {
        moveArm(-ticks, speed);
    }

    private void ArmUp (int ticks, double speed) {
        moveArm(ticks, speed);
    }

    private void Scoop (int ScoopMotorTicks, double speed) {
        ScoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ScoopMotor.setTargetPosition(ScoopMotorTicks);

        ScoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ScoopMotor.setPower(speed);

    }

    private void ScoopIn (int ticks, double speed) {
        Scoop(-ticks, speed);
    }

    private void ScoopOut (int ticks, double speed) {
        Scoop(ticks, speed);
    }



}


