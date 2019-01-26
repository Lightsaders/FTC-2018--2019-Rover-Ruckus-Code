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
@TeleOp(name = "Single_Player", group = "TeleOp")
@Disabled
public class Single_Player extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("frontLeftDrive");
        driveFrontRight = hardwareMap.dcMotor.get("frontRightDrive");
        driveBackLeft = hardwareMap.dcMotor.get("backLeftDrive");
        driveBackRight = hardwareMap.dcMotor.get("backRightDrive");
        outtakeSlide = hardwareMap.dcMotor.get("outtakeSlide");
        intakeSlide = hardwareMap.dcMotor.get("intakeSlide");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        //intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

//            telemetry.addData("FrontLeftMotorPower", driveFrontLeft.getPower());
//            telemetry.addData("FrontRightMotorPower", driveFrontRight.getPower());
//            telemetry.addData("BackRightMotorPower", driveBackLeft.getPower());
//            telemetry.addData("BackLeftMotorPower", driveBackRight.getPower());
            telemetry.addData("OuttakeLiftMotorPower", outtakeSlide.getPower());
            telemetry.addData("IntakeSlideMotorPower", intakeSlide.getPower());
            telemetry.addData("LiftMotorPower", liftMotor.getPower());
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

            liftMotor.setPower(gamepad1.right_stick_y);

            if(gamepad1.left_bumper){
                intakeSlide.setPower(0.75);
            }else if(gamepad1.right_bumper) {
                intakeSlide.setPower(-0.75);
            }else{
                intakeSlide.setPower(0.0);
            }


            if(gamepad1.left_trigger>0){
                outtakeSlide.setPower(0.75);
            }else if(gamepad1.right_trigger>0) {
                outtakeSlide.setPower(-0.75);
            }else{
                outtakeSlide.setPower(0.0);
            }
        }
        idle();
    }
}