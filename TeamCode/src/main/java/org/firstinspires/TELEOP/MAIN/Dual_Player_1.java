package org.firstinspires.TELEOP.MAIN;

import android.sax.TextElementListener;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;


/**
 * Created by robotics on 10/30/2017.
 */
@TeleOp(name = "RYLAN-BASE_GRANT-EXTREMA", group = "TeleOp")
public class Dual_Player_1 extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private DcMotor liftMotor;
    private DcMotor intake;

    private CRServo crunchLeft;
    private CRServo crunchRight;
    private Servo outtake;


    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
        intakeSlide = hardwareMap.dcMotor.get("intakeSlide");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        intake = hardwareMap.dcMotor.get("intake");

        crunchLeft = hardwareMap.crservo.get("crunchLeft");
        crunchRight = hardwareMap.crservo.get("crunchRight");
        outtake = hardwareMap.servo.get("outtake");

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        outtake.setPosition(0.41);

        while (opModeIsActive()) {

            telemetry.addData("Current Time: ", getRuntime());
            telemetry.update();

            // GAMEPAD 1 BASE

            // LEFT STICK Y - FORWARDS AND BACKWARDS
            driveFrontLeft.setPower(gamepad1.left_stick_y);
            driveBackLeft.setPower(gamepad1.left_stick_y);
            driveFrontRight.setPower(gamepad1.left_stick_y);
            driveBackRight.setPower(gamepad1.left_stick_y);

            // LEFT STICK X - STRAFE LEFT AND RIGHT
            driveFrontLeft.setPower(gamepad1.left_stick_x*-1);
            driveBackLeft.setPower(gamepad1.left_stick_x);
            driveFrontRight.setPower(gamepad1.left_stick_x);
            driveBackRight.setPower(gamepad1.left_stick_x*-1);

            // RIGHT STICK X - TURN CLOCKWISE AND COUNTERCLOCKWISE
            driveFrontLeft.setPower(gamepad1.right_stick_x * -1);
            driveBackLeft.setPower(gamepad1.right_stick_x * -1);
            driveFrontRight.setPower(gamepad1.right_stick_x);
            driveBackRight.setPower(gamepad1.right_stick_x);

            if(gamepad1.right_bumper){
                outtakeSlide.setPower(-1.0);
            } else if(gamepad1.left_bumper){
                outtakeSlide.setPower(1.0);
            }else{
                outtakeSlide.setPower(0);
            }

            if(gamepad1.a){// outtake receive
                outtake.setPosition(0.9);
            }
            if(gamepad1.b){// outtake dump
                outtake.setPosition(0.43);
            }
            if(gamepad1.x){// outtake intermediary 1
                outtake.setPosition(0.5);
            }
            if(gamepad1.y){// outtake intermediary 2
                outtake.setPosition(0.75);
            }


            // GAMEPAD 2 EXTREMITIES

            liftMotor.setPower(gamepad2.right_stick_y*-1);
            intakeSlide.setPower(gamepad2.left_stick_x);

            if(gamepad2.a){// stop
                intake.setPower(0.0);
            }
            if(gamepad2.x){// reverse
                intake.setPower(-1.0);
            }
            if(gamepad2.b){// forward
                intake.setPower(1.0);
            }

            if(gamepad2.left_bumper){// crunch UP
                crunchLeft.setPower(1);
                crunchRight.setPower(-1);
            } else if(gamepad2.right_bumper){// crunch DOWN
                crunchLeft.setPower(-1);
                crunchRight.setPower(1);
            } else{
                crunchLeft.setPower(0);
                crunchRight.setPower(0);
            }
        }
        idle();
    }
}