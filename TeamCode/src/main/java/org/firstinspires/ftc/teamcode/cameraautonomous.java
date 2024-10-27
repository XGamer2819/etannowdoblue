

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name="cameraautonomous", group="Linear OpMode")

public class cameraautonomous extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor Slide;
    private DcMotor SlideRotation;
    private CRServo Intake;













    //init motors, servo, camera and color sensor
    private void initDcMotors() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        Slide = hardwareMap.get(DcMotor.class, "slide");
        SlideRotation =  hardwareMap.get(DcMotor.class, "slide rotation");
        Intake = hardwareMap.get(CRServo.class, "intake");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }



    //stop drive
    private void driveStop() {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
    private void slideRotateStop() {
        SlideRotation.setPower(0);
    }
    private void slideStop() {
        Slide.setPower(0);
    }
    private void IntakeStop() {
        Intake.setPower(0);
    }
    private void SlideMove(String direction, double runtimeinseconds, double motorpower) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Slide.setDirection(DcMotor.Direction.REVERSE);
        if (direction == "Extend") {
            Slide.setPower(motorpower);
        }
        if (direction == "UnExtend") {
            Slide.setPower(-motorpower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void WheelIntake(String direction, double runtimeinseconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Intake.setDirection(CRServo.Direction.FORWARD);
        if (direction == "Outake") {
            Intake.setPower(-1);
        }
        if (direction == "Intake") {
            Intake.setPower(1);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void slideRotationMove(String direction, double runtimeinseconds, double MotorPower) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
       // SlideRotation.setDirection(DcMotor.Direction.REVERSE);
        if (direction == "Up") {
            SlideRotation.setPower(-MotorPower);
        }
        if (direction == "Down") {
            SlideRotation.setPower(MotorPower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeinseconds)) {
            telemetry.update();
        }
    }

    private void drive(String direction, double runtimeInseconds,
                       double leftFrontPower, double rightFrontPower,
                       double leftBackPower, double rightBackPower) {
        ElapsedTime runtime = new ElapsedTime();
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        if (direction == "Forward") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

        }
        if (direction == "Backward") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);

        }
        if (direction == "Left") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);

        }
        if (direction == "Right") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }
        if (direction == "TurnRight") {
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(-rightFrontPower);
            rightBackDrive.setPower(-rightBackPower);
        }
        if (direction == "TurnLeft") {
            leftFrontDrive.setPower(-leftFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }

        while (opModeIsActive() && (runtime.seconds() < runtimeInseconds)) {
            telemetry.update();

        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        initDcMotors();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            while (opModeIsActive()) { 
            slideRotationMove("Up",3.15, 1);
            slideRotateStop();
           drive("Forward", 1, 0.5, 0.5, 0.5, 0.5);
           driveStop();
           drive("Left", 1, 0.5, 0.5, 0.5, 0.5 );
           driveStop();
           drive("TurnLeft", 1.5, 0.5, 0.5, 0.5, 0.5);
           drive("Forward", 0.8, 0.5, 0.5, 0.5, 0.5);
            driveStop();
           SlideMove("Extend", 1.75, 1);
           WheelIntake("Outake", 1.5);
            IntakeStop();
            driveStop();
            SlideMove("UnExtend", 0.25, 0.5);
            slideStop();
            telemetry.update();
            break;






            }
        }


                }

        }

