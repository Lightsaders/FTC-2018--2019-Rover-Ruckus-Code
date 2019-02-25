package org.firstinspires.AUTOMOUS;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "RESET")
public class RESET extends LinearOpMode {

    private DcMotor liftMotor;
    private Servo outtake;
    private Servo marker;

    // Lift Motor Specs
    double COUNTS_PER_MOTOR_LIFT = 388;
    double CM_PPR_LIFT = COUNTS_PER_MOTOR_LIFT / 0.8;


    @Override
    public void runOpMode() throws InterruptedException {

        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        outtake = hardwareMap.servo.get("outtake");
        marker = hardwareMap.servo.get("marker");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake.setPosition(0.45);
        marker.setPosition(0.62);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status: ", "waiting for start command");
            telemetry.update();
        }

        if (opModeIsActive()) {

            landing(-17, 1.0);

        }
    }

    public void landing(double distanceCM, double speed) {
        int target;

        if (opModeIsActive()) {

            liftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

            // Determine new target position, and pass to motor controller
            target = liftMotor.getCurrentPosition() + (int) (distanceCM * CM_PPR_LIFT);

            // set target position to motor
            liftMotor.setTargetPosition(target);

            // Turn on run to position
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(Math.abs(speed));

            // loop while position is not reached
            while (opModeIsActive() && (liftMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("VERTICAL LINEAR SLIDE MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", target, liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            liftMotor.setPower(0);

            //Turn off run to position
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
