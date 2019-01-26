package org.firstinspires.TELEOP.TEST_ROBOT;

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
@TeleOp(name = "TEST_TELEOP", group = "TeleOp")
@Disabled
public class TEST_TELEOP extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;

    @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            driveFrontLeft.setPower(gamepad1.left_stick_y);
            driveBackLeft.setPower(gamepad1.left_stick_y);
            driveFrontRight.setPower(gamepad1.left_stick_y);
            driveBackRight.setPower(gamepad1.left_stick_y);

            driveFrontLeft.setPower(gamepad1.left_stick_x*-1);
            driveBackLeft.setPower(gamepad1.left_stick_x);
            driveFrontRight.setPower(gamepad1.left_stick_x);
            driveBackRight.setPower(gamepad1.left_stick_x*-1);

            driveFrontLeft.setPower(gamepad1.right_stick_x*-1);
            driveBackLeft.setPower(gamepad1.right_stick_x*-1);
            driveFrontRight.setPower(gamepad1.right_stick_x);
            driveBackRight.setPower(gamepad1.right_stick_x);

        }
        idle();
    }
}