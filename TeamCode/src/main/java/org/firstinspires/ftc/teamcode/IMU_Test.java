package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IMU-PID Movement")
public class IMU_Test extends LinearOpMode {

    // Hardware
    private DcMotor BackLeftDrive, BackRightDrive, FrontLeftDrive, FrontRightDrive, ScoopMotor;
    private BHI260IMU imu;

    // PID Controllers
    private PIDController stabilizationPID = new PIDController(0.01, 0, 0.005); // Adjust constants as needed

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // IMU Initialization
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imu.initialize(new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example Autonomous Path
            Forward(1000, 0.7); // Move forward 1000 ticks at 70% speed
            StrafeRight(500, 0.7); // Strafe right 500 ticks
            DiagonalForwardLeft(700, 0.6); // Move diagonally forward-left
            Backward(1000, 0.7); // Move backward
            RunScoopMotorForTime(0.5, 2000); // Run scoop motor for 2 seconds
            TurnToAngle(90,0.5);

        }
    }

    // Movement methods
    private void Forward(int ticks, double speed) {
        moveRobotIMU(ticks, ticks, ticks, ticks, speed);
    }

    private void Backward(int ticks, double speed) {
        moveRobotIMU(-ticks, -ticks, -ticks, -ticks, speed);
    }

    private void StrafeRight(int ticks, double speed) {
        moveRobotIMU(-ticks, ticks, ticks, -ticks, speed);
    }

    private void StrafeLeft(int ticks, double speed) {
        moveRobotIMU(ticks, -ticks, -ticks, ticks, speed);
    }

    private void DiagonalForwardLeft(int ticks, double speed) {
        moveRobotIMU(0, ticks, ticks, 0, speed);
    }

    private void DiagonalForwardRight(int ticks, double speed) {
        moveRobotIMU(ticks, 0, 0, ticks, speed);
    }

    private void DiagonalBackwardLeft(int ticks, double speed) {
        moveRobotIMU(-ticks, 0, 0, -ticks, speed);
    }

    private void DiagonalBackwardRight(int ticks, double speed) {
        moveRobotIMU(0, -ticks, -ticks, 0, speed);
    }

    // General movement with IMU-based stabilization
    private void moveRobotIMU(int backLeftTicks, int backRightTicks, int frontLeftTicks, int frontRightTicks, double maxSpeed) {
        // Reset encoders
        resetDriveEncoders();

        // Set target positions
        BackLeftDrive.setTargetPosition(backLeftTicks);
        BackRightDrive.setTargetPosition(backRightTicks);
        FrontLeftDrive.setTargetPosition(frontLeftTicks);
        FrontRightDrive.setTargetPosition(frontRightTicks);

        // Set motors to RUN_USING_ENCODER mode
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Capture the desired heading
        double desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        boolean moving = true;
        while (opModeIsActive() && moving) {
            // Get current yaw
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate stabilization correction
            double correction = stabilizationPID.calculate(desiredHeading, currentYaw);
            correction = Range.clip(correction, -maxSpeed * 0.2, maxSpeed * 0.2);

            // Calculate power for each side
            double leftPower = maxSpeed - correction;
            double rightPower = maxSpeed + correction;

            // Clip power
            leftPower = Range.clip(leftPower, -maxSpeed, maxSpeed);
            rightPower = Range.clip(rightPower, -maxSpeed, maxSpeed);

            // Set motor powers
            BackLeftDrive.setPower(backLeftTicks < 0 ? -leftPower : leftPower);
            BackRightDrive.setPower(backRightTicks < 0 ? -rightPower : rightPower);
            FrontLeftDrive.setPower(frontLeftTicks < 0 ? -leftPower : leftPower);
            FrontRightDrive.setPower(frontRightTicks < 0 ? -rightPower : rightPower);

            // Telemetry
            telemetry.addData("Target Heading", desiredHeading);
            telemetry.addData("Current Yaw", currentYaw);
            telemetry.addData("Correction", correction);
            telemetry.update();

            // Check if motors have reached their targets
            moving = Math.abs(BackLeftDrive.getCurrentPosition() - backLeftTicks) > 10 ||
                    Math.abs(BackRightDrive.getCurrentPosition() - backRightTicks) > 10 ||
                    Math.abs(FrontLeftDrive.getCurrentPosition() - frontLeftTicks) > 10 ||
                    Math.abs(FrontRightDrive.getCurrentPosition() - frontRightTicks) > 10;
        }

        stopMotors();
    }

    // Time-based scoop motor control
    private void RunScoopMotorForTime(double power, long durationMillis) {
        ScoopMotor.setPower(power);
        sleep(durationMillis);
        ScoopMotor.setPower(0);
    }

    // Stop all motors
    private void stopMotors() {
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
    }

    // Reset drivetrain motor encoders
    private void resetDriveEncoders() {
        DcMotor[] motors = {BackLeftDrive, BackRightDrive, FrontLeftDrive, FrontRightDrive};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Initialize hardware
    private void initializeHardware() {
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        ScoopMotor = hardwareMap.get(DcMotor.class, "ScoopMotor");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Motor directions
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }


    public class PIDController {
        private double kP, kI, kD;
        private double previousError = 0;
        private double integralSum = 0;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double target, double current) {
            double error = target - current;

            // Proportional term
            double proportional = kP * error;

            // Integral term
            integralSum += error;
            double integral = kI * integralSum;

            // Derivative term
            double derivative = kD * (error - previousError);
            previousError = error;

            // PID output
            return proportional + integral + derivative;
        }

        // Optionally reset the integral and previous error
        public void reset() {
            integralSum = 0;
            previousError = 0;
        }
    }

    private void TurnToAngle(double targetAngle, double maxSpeed) {
        // Reset PID controller
        stabilizationPID.reset();

        // Ensure the target angle is in the range -180 to 180
        targetAngle = normalizeAngle(targetAngle);

        // Loop until the robot reaches the target angle
        while (opModeIsActive()) {
            // Get the current yaw from the IMU
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the error (difference between target and current angle)
            double error = normalizeAngle(targetAngle - currentYaw);

            // Check if the robot is within the acceptable range
            if (Math.abs(error) < 1.0) { // 1 degree threshold
                break;
            }

            // Get the correction from the PID controller
            double correction = stabilizationPID.calculate(targetAngle, currentYaw);
            correction = Range.clip(correction, -maxSpeed, maxSpeed);

            // Apply correction to motors for turning
            BackLeftDrive.setPower(correction);
            BackRightDrive.setPower(-correction);
            FrontLeftDrive.setPower(correction);
            FrontRightDrive.setPower(-correction);

            // Telemetry for debugging
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Yaw", currentYaw);
            telemetry.addData("Error", error);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop the motors after the turn
        stopMotors();
    }

    // Helper function to normalize angles to the range -180 to 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }







}
