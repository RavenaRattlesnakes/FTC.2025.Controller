package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2025 DRIVER MODE")
public class TeleOpTest extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor FrontLeftDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackRightDrive = null;
    public DcMotor MoonMotor = null;
    public DcMotor ScoopMotor = null;
    public DcMotor ArmMotor = null;
    public Servo LeftServo = null;
    public Servo RightServo = null;
    public Servo MiddleServo = null;


    //Used for keeping linear slide extended
    double targetPosition = 0;   // Desired encoder position
    double kP = 0.01;           // Proportional constant (adjust if not working)
    double error = 0;           // Difference between target and current position
    double motorPower = 0;      // Power sent to the motor


    @Override
    public void runOpMode() {

        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeftDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        MoonMotor = hardwareMap.get(DcMotor.class, "MoonMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ScoopMotor = hardwareMap.get(DcMotor.class, "ScoopMotor");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");
        MiddleServo = hardwareMap.get(Servo.class, "MiddleServo");

        MoonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Reset encoders
        MoonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           //Set the encoder to manual control
        MoonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Brake mode
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        ScoopMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftServo.setPosition(1);
        RightServo.setPosition(0);
        MiddleServo.setPosition(0.9);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            FrontLeftDrive.setPower(leftFrontPower);
            FrontRightDrive.setPower(rightFrontPower);
            BackLeftDrive.setPower(leftBackPower);
            BackRightDrive.setPower(rightBackPower);

            double currentPosition = MoonMotor.getCurrentPosition();

            if (gamepad2.b) {               //slide go down
                MoonMotor.setPower(0.40);
                targetPosition = currentPosition;

            } else if (gamepad2.a) {        //slide go up
                MoonMotor.setPower(-0.40);
                targetPosition = currentPosition;

            } else {
                error = targetPosition - currentPosition;     //Calculate error
                motorPower = error * kP;                      //P in PID
                MoonMotor.setPower(motorPower);               //Apply power

            }


            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

            if (gamepad2.x) {
                ArmMotor.setPower(0.6);

            } else if (gamepad2.y) {
                ArmMotor.setPower(-0.6);

            } else {
                ArmMotor.setPower(0);

            }

            if(gamepad2.left_bumper) {         // close servos
                RightServo.setPosition(0);
                LeftServo.setPosition(1);
            }

            if(gamepad2.dpad_up) {             //take in sample
                ScoopMotor.setPower(0.8);

            } else if(gamepad2.dpad_down) {     //spit out sample
                ScoopMotor.setPower(-1.0);

            }else {
                ScoopMotor.setPower(0);

            }

            if(gamepad2.right_bumper) {           // open servos
                RightServo.setPosition(0.45);
                LeftServo.setPosition(0.45);
            }

            if(gamepad1.a) {                      // dump basket
                MiddleServo.setPosition(0);
            }

            if (gamepad1.b){                      // return basket
                MiddleServo.setPosition (0.87);

            }



        }
    }
}
