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

@Autonomous(name = "AutoTest")
public class AutonomousTest extends LinearOpMode {

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

        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Initialize IMU parameters
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        
        imu.initialize(parameters);

        // Motor configuration
        MoonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MoonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MoonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        ScoopMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftServo.setPosition(0);
        RightServo.setPosition(1);
        MiddleServo.setPosition(0.9);

        resetEncoders();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example pathway
            Forward(1000, 0.5);    // Move forward
            Backward(1000, 0.5);   // Move backward
            StrafeRight(500, 0.5); // Strafe right
            StrafeLeft(500, 0.5);  // Strafe left
            TurnLeft(90, 0.5);     // Turn left 90 degrees
            TurnRight(90, 0.5);    // Turn right 90 degrees
        }
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

    private void StrafeLeft(int ticks, double speed) {
        moveRobot(ticks, -ticks, -ticks, ticks, speed);
    }

    private void stopMotors() {
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
    }

    private void moveRobot(int backLeftTicks, int backRightTicks, int frontLeftTicks, int frontRightTicks, double speed) {
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    private void TurnLeft(double degrees, double power) {
        turnToAngle(degrees, power, true);
    }

    // Turn the robot right by a specified angle
    private void TurnRight(double degrees, double power) {
        turnToAngle(degrees, power, false);
    }

    private void turnToAngle(double targetAngle, double power, boolean isLeftTurn) {
        double initialAngle = getAngle();
        double target = initialAngle + (isLeftTurn ? -targetAngle : targetAngle);
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && Math.abs(target - getAngle()) > 1 && runtime.seconds() < 5) {
            double turnPower = Range.clip(power, -1.0, 1.0);
            if (isLeftTurn) {
                FrontLeftDrive.setPower(-turnPower);
                FrontRightDrive.setPower(turnPower);
                BackLeftDrive.setPower(-turnPower);
                BackRightDrive.setPower(turnPower);
            } else {
                FrontLeftDrive.setPower(turnPower);
                FrontRightDrive.setPower(-turnPower);
                BackLeftDrive.setPower(turnPower);
                BackRightDrive.setPower(-turnPower);
            }
        }

        // Stop the motors
        stopMotors();
    }

    // Get the current angle from the IMU
    private double getAngle() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}