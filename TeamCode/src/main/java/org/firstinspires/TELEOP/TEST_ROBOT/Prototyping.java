package org.firstinspires.TELEOP.TEST_ROBOT;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name = "Roberto v2", group = "Teleop")
@Disabled
public class Prototyping extends LinearOpMode {

    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
//    private DistanceSensor rightEyem;
//    private DistanceSensor leftEyemM;

    double COUNTS_PER_MOTOR_REV = 1120;    // using REV HD 40:1
    double DRIVE_GEAR_REDUCTION = 0.5;    // 80 tooth to 40 tooth
    double WHEEL_DIAMETER_CM = 7.79;     // mecanum wheels
    double TUNING_DRIVE = 1.75;
    double ROBOT_RADIUS_CM = 21;
    double COUNTS_PER_CM_REV = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * TUNING_DRIVE) / (WHEEL_DIAMETER_CM * Math.PI)) / 2;

    // The IMU sensor object
    BNO055IMU imu;
    public double heading;
    ModernRoboticsI2cRangeSensor leftEyem;
    ModernRoboticsI2cRangeSensor rightEyem;
    

    

        @Override
    public void runOpMode() throws InterruptedException {

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        imu = (BNO055IMU) hardwareMap.get("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

            // get a reference to our compass
            leftEyem = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftEyem");

            rightEyem = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightEyem");


//        rightEyem = hardwareMap.get(DistanceSensor.class, "rightEyem");
//        Rev2mDistanceSensor sensorTimeOfFlightRight = (Rev2mDistanceSensor) rightEyem;
//        leftEyemM = hardwareMap.get(DistanceSensor.class, "leftEyemM");
//        Rev2mDistanceSensor sensorTimeOfFlightLeft = (Rev2mDistanceSensor) leftEyemM;

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        double leftSum = 0;
        double rightSum = 0;
        int count = 0;
        double leftAverage = 0;
        double rightAverage = 0;

        while (opModeIsActive()) {
//            count++;
//            leftSum = leftSum + leftEyem.getDistance(DistanceUnit.CM);
//            rightSum = rightSum + rightEyem.getDistance(DistanceUnit.CM);
//            leftAverage = leftSum / count;
//            rightAverage = rightSum / count;
//            telemetry.addData("rightEyem", rightEyem.getDistance(DistanceUnit.CM));
//            telemetry.addData("RightAverage", rightAverage);
//            telemetry.addData("leftEyemM", leftEyem.getDistance(DistanceUnit.CM));
//            telemetry.addData("LeftAverage", leftAverage);
//
//            telemetry.update();
        alignWall();
              sleep(2000);
        }
        idle();
    }

    public void alignWall() {
        double larger = 0;
        double smaller = 0;
        String turn = "";
        if (rightEyem.getDistance(DistanceUnit.CM) > leftEyem.getDistance(DistanceUnit.CM)) {
            larger = rightEyem.getDistance(DistanceUnit.CM);
            smaller = leftEyem.getDistance(DistanceUnit.CM);
            turn = "CC";

        } else if (rightEyem.getDistance(DistanceUnit.CM) < leftEyem.getDistance(DistanceUnit.CM)) {
            larger = leftEyem.getDistance(DistanceUnit.CM);
            smaller = rightEyem.getDistance(DistanceUnit.CM);
            turn = "C";
        } else {
            telemetry.addData("SomethingisWrong", "");
            telemetry.update();
            sleep(2000);
        }
        double x = 24.6;
        double y = larger - smaller;
        double angle = Math.atan2(y, x);
        angle = Math.toDegrees(angle);
        telemetry.addData("larger", larger);
        telemetry.addData("Smaller", smaller);
        telemetry.addData("Y", y);
        telemetry.addData("Angle", angle);
        telemetry.update();
//        sleep(2000);
        turnEncoder(0.5, angle, turn);

    }

    //    public void IMUturn(double degree, String direction, double speed, double tolerance) {
//
//        double initialHeading;
//        double finalHeading;
//
//        switch (direction) {
//            case "C":// clockwise
//
//                initialHeading = getCurrentHeading();
//                finalHeading = initialHeading + degree;
//                if (finalHeading > 360) {
//                    finalHeading = finalHeading - 360;
//                }
//
//                while (opModeIsActive() && (int) (getCurrentHeading()) < (int) (finalHeading + tolerance)) {
//                    driveFrontLeft.setPower(speed);
//                    driveFrontRight.setPower(speed * -1);
//                    driveBackLeft.setPower(speed);
//                    driveBackRight.setPower(speed * -1);
//                    telemetry.addData("1CURRENT HEADING: ", "" + getCurrentHeading() + " " + finalHeading);
//                    telemetry.update();
//                }
//
//                driveFrontLeft.setPower(0);
//                driveFrontRight.setPower(0);
//                driveBackLeft.setPower(0);
//                driveBackRight.setPower(0);
//
//                break;
//            case "CC":// counterclockwise
//
//                initialHeading = getCurrentHeading();
//                finalHeading = initialHeading - degree;
//                if (finalHeading < 0) {
//                    finalHeading = finalHeading + 360;
//                }
//
//                while (opModeIsActive() && (int) (getCurrentHeading()) > (int) (finalHeading - tolerance)) {
//                    driveFrontLeft.setPower(speed * -1);
//                    driveFrontRight.setPower(speed);
//                    driveBackLeft.setPower(speed * -1);
//                    driveBackRight.setPower(speed);
//                    telemetry.addData("2CURRENT HEADING: ", "" + getCurrentHeading() + " " + finalHeading);
//                    telemetry.update();
//                }
//
//                driveFrontLeft.setPower(0);
//                driveFrontRight.setPower(0);
//                driveBackLeft.setPower(0);
//                driveBackRight.setPower(0);
//
//                break;
//        }
//    }
//
//public double getCurrentHeading() {
//    heading = 0.0;
//    heading = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle * -1);
//    if (heading < 0) {
//        heading = heading + 360;
//
//    }
//    return heading;
//}
//
    public void turnEncoder(double speed, double turnDegrees, String direction) {
        double tuning = 1;
        double distance = ROBOT_RADIUS_CM * tuning* (((turnDegrees) * (Math.PI)) / (180)); // Using arc length formula
        int frontLeftTarget = 0;
        int backLeftTarget = 0;
        int frontRightTarget = 0;
        int backRightTarget = 0;
        double end = 0;
        double t = 0;

        //RESET ENCODERS
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (direction) {
            case "C":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                break;
            case "CC":
                // Determine new target position, and pass to motor controller
                frontLeftTarget = driveFrontLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                frontRightTarget = driveFrontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                backLeftTarget = driveBackLeft.getCurrentPosition() + (int) (distance * -1 * COUNTS_PER_CM_REV);
                backRightTarget = driveBackRight.getCurrentPosition() + (int) (distance * COUNTS_PER_CM_REV);
                break;
        }

        if (opModeIsActive()) {

            // set target position to each motor
            driveFrontLeft.setTargetPosition(frontLeftTarget);
            driveFrontRight.setTargetPosition(frontRightTarget);
            driveBackLeft.setTargetPosition(backLeftTarget);
            driveBackRight.setTargetPosition(backRightTarget);

            // Turn on run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveFrontLeft.setPower(speed);
            driveFrontRight.setPower(speed);
            driveBackLeft.setPower(speed);
            driveBackRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            //t= getRuntime();
            //end = (Math.abs(turnDegrees)/10.16)/(speed/0.05) + getRuntime();

            while (opModeIsActive() &&
                    //(getRuntime() <= end)&&
                    (driveFrontLeft.isBusy() || driveFrontRight.isBusy() || driveBackLeft.isBusy() || driveBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("RUN TIME CURRENT: ", ""+getRuntime());
                telemetry.addData("RUN TIME END: ", ""+end);
                telemetry.addData("FRONT LEFT MOTOR", " DRIVING TO: %7d CURRENTLY AT: %7d", frontLeftTarget, driveFrontLeft.getCurrentPosition());
                telemetry.addData("FRONT RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", frontRightTarget, driveFrontRight.getCurrentPosition());
                telemetry.addData("BACK LEFT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backLeftTarget, driveBackLeft.getCurrentPosition());
                telemetry.addData("BACK RIGHT MOTOR", "DRIVING TO: %7d CURRENTLY AT: %7d", backRightTarget, driveBackRight.getCurrentPosition());
                telemetry.update();
            }
            telemetry.clearAll();
            telemetry.addData("FINISHED RUN: ", ""+(end-t));
            telemetry.update();

            // Stop all motion;
            driveFrontLeft.setPower(0);
            driveFrontRight.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);

            //Turn off run to position
            driveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        //telemetrySender("DEGREES CURRENT: ", "" + getCurrentHeading(), "");
        //telemetrySender("DEGREES FINAL: ", "" + (getCurrentHeading() + headingStart), "");
    }
}
