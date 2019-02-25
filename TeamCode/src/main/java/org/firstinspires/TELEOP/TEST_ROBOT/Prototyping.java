package org.firstinspires.TELEOP.TEST_ROBOT;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name = "Prototyping", group = "Teleop")

public class Prototyping extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Drive Motors
        motor = hardwareMap.dcMotor.get("motor");


        waitForStart();

        while (opModeIsActive()) {
                motor.setPower(gamepad1.left_stick_y);
        }
        idle();
    }
}