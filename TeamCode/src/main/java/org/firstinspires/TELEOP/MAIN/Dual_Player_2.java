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
@TeleOp(name = "GRANT-BASE-RYLAN-EXTREMA", group = "TeleOp")
@Disabled
public class Dual_Player_2 extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;

    private Servo marker;
    private Servo outtake;
    private CRServo intake;

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
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        marker = hardwareMap.servo.get("marker");
        outtake = hardwareMap.servo.get("outtake");
        intake = hardwareMap.crservo.get("intake");

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
            driveFrontLeft.setPower(gamepad1.right_stick_x*-1);
            driveBackLeft.setPower(gamepad1.right_stick_x*-1);
            driveFrontRight.setPower(gamepad1.right_stick_x);
            driveBackRight.setPower(gamepad1.right_stick_x);

            // GAMEPAD 2 EXTREMITIES

            // LEFT TRIGGER  - LIFT MOTOR UP
            liftMotor.setPower(gamepad2.left_trigger*-1);
            // RIGHT TRIGGER  - LIFT MOTOR DOWN
            liftMotor.setPower(gamepad2.right_trigger);

            // LEFT STICK Y - INTAKE SLIDE
            intakeSlide.setPower(gamepad2.left_stick_y);

            // RIGHT STICK Y - OUTTAKE SLIDE
            outtakeSlide.setPower(gamepad2.right_stick_y*-1);

            // LETTER BUTTONS - OUTTAKE POSITION
            // Y BUTTON DUMP
            if(gamepad2.y) {
                outtake.setPosition(0.8);
            }
            // A BUTTON RECEIVE
            if(gamepad2.a){
                outtake.setPosition(0.8);
            }
            // B BUTTON PARALLEL
            if(gamepad2.dpad_right){
                outtake.setPosition(0.5);
            }

            // DPAD - INTAKE POSITION
            // DOWN BUTTON DOWN
            while(gamepad2.dpad_down) {
                intakeMotor.setPower(-1.0);
            }
            // UP BUTTON UP
            while(gamepad2.dpad_up){
                intakeMotor.setPower(1.0);
            }

            // INTAKE DIRECTION
            // LEFT BUTTON INTAKE
            if(gamepad2.dpad_left){
                intake.setPower(1.0);
            }
            // RIGHT BUTTON OUTTAKE
            if(gamepad2.dpad_right){
                intake.setPower(-1.0);
            }
            // X BUTTON STOP
            if(gamepad2.x){
                intake.setPower(0);
            }

        }
        idle();
    }
}