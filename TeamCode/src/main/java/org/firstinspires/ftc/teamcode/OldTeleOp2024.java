package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OldTeleOp2024")
@Disabled
public class OldTeleOp2024 extends LinearOpMode {

    public DcMotor BackRight;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor FrontLeft;
    // public DcMotor Flag;
    //public Servo Flag;
    //public DcMotor LinearSlide;
    //public Servo RClaw;
    //public Servo LClaw;

    @Override
    public void runOpMode() {
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        // DcMotorEx LinearSlide = hardwareMap.get(DcMotorEx.class, "Linear Slide");
        //Flag = hardwareMap.get(Servo.class, "Flag");
        //LClaw = hardwareMap.get(Servo.class, "LClaw");

        int z = 0;


        //LinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //brake

        while (opModeIsActive()) {
            //for buttons
            if (gamepad2.dpad_up) {
                z = -7000;
            }
            if (gamepad2.dpad_down) {
                z = 0;
                // LinearSlide.setVelocity(4000); }







//auto placements
                //short


                //  LinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
                // LinearSlide.setTargetPosition(z);
                //LinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                //break program
                if (gamepad1.back || gamepad2.back) {
                    break;
                }

                FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                //speeds ** change this/?
                double y = -gamepad1.left_stick_y * 30000;
                double ry = -gamepad1.right_stick_y * 30000;
                double x = -gamepad1.left_stick_x * 30000;
                double rx = -gamepad1.right_stick_x * 30000;


                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double rightFrontPower = (y+x+rx+ry * 30) * 30 / denominator;
                double rightBackPower =  (y-x+rx+ry * 30) * 30 / denominator;
                double leftFrontPower =  (y-x-rx+ry * 30) *30 / denominator;
                double leftBackPower =  (y+x-rx+ry * 30) * 30 / denominator;



                FrontRight.setPower(rightFrontPower * 30);
                BackRight.setPower(rightBackPower * 30);
                FrontLeft.setPower(leftFrontPower * 30);
                BackLeft.setPower(leftBackPower * 30);

            }
        }
    }
}