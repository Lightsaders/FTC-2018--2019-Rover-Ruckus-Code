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

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("FrontLeftMotorPower", driveFrontLeft.getPower());
            telemetry.addData("FrontRightMotorPower", driveFrontRight.getPower());
            telemetry.addData("BackRightMotorPower", driveBackLeft.getPower());
            telemetry.addData("BackLeftMotorPower", driveBackRight.getPower());
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

            outtakeSlide.setPower(gamepad1.left_trigger);
            outtakeSlide.setPower(gamepad1.right_trigger*-2);

            if(gamepad1.a){// dump
                outtake.setPosition(0.9);// this
            }
            if(gamepad1.b){// standard position
                outtake.setPosition(0.41);// this
            }
            if(gamepad1.x){
                outtake.setPosition(0.5);// this
            }
            if(gamepad1.y){
                outtake.setPosition(0.75);// this
            }

            // Grant Controls
          
            intakeSlide.setPower(gamepad2.left_stick_y);
            crunchLeft.setPower(gamepad2.right_stick_y);
            crunchRight.setPower(gamepad2.right_stick_y*-1);
            if(gamepad2.a){
                intake.setPower(1.0);
            }
            if(gamepad2.b){
                intake.setPower(-1.0);
            }
            if(gamepad2.x){
                intake.setPower(0.0);
            }
            liftMotor.setPower(gamepad2.left_trigger);
            liftMotor.setPower(gamepad2.right_trigger*-2);

            
            

        }
        idle();
    }
}