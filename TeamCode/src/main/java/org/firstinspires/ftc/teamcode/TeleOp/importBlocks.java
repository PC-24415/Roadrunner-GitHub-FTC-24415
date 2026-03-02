package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp(name = "REDTeamCode (Blocks to Java)")
public class importBlocks extends LinearOpMode {

    private RevBlinkinLedDriver Blinkin;
    private DcMotor LeftFront;
    private DcMotor LeftRear;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private DcMotor Intake;
    private DcMotor launcherright;
    private DcMotor Turret;
    private DcMotor launcherleft;
    private Servo Lifter;
    private Servo rightstopper;
    private Servo leftstopper;
    private Servo leftleg;
    private Servo rightleg;

    AprilTagProcessor myAprilTagProcessor;
    boolean USE_WEBCAM;

    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Set the camera to the specified webcam name.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        // Add the AprilTag processor.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Build the VisionPortal object and assign it to a variable.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private void InitializeBlinkinSystem() {
        RevBlinkinLedDriver.BlinkinPattern Pattern;
        boolean autoDisplay;

        Pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        Blinkin.setPattern(Pattern);
        autoDisplay = true;
        telemetry.addData("Auto Display Mode", autoDisplay);
        telemetry.addData("Pattern", Pattern);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        boolean Last_Position_of_X;
        boolean Last_Position_of_B;
        boolean Last_Position_of_A;
        boolean Last_Position_of_Y;
        boolean intakeOn;
        float axial;
        float lateral;
        float yaw;
        float frontLeftPower;
        float frontRightPower;
        float backLeftPower;
        float backRightPower;

        Blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        LeftFront = hardwareMap.get(DcMotor.class, "Left Front");
        LeftRear = hardwareMap.get(DcMotor.class, "Left Rear");
        RightFront = hardwareMap.get(DcMotor.class, "Right Front");
        RightRear = hardwareMap.get(DcMotor.class, "Right Rear");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        launcherright = hardwareMap.get(DcMotor.class, "launcher right");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        launcherleft = hardwareMap.get(DcMotor.class, "launcher left");
        Lifter = hardwareMap.get(Servo.class, "Lifter");
        rightstopper = hardwareMap.get(Servo.class, "right stopper");
        leftstopper = hardwareMap.get(Servo.class, "left stopper");
        leftleg = hardwareMap.get(Servo.class, "left leg");
        rightleg = hardwareMap.get(Servo.class, "right leg");

        runtime = new ElapsedTime();
        USE_WEBCAM = true;
        initAprilTag();
        InitializeBlinkinSystem();
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        launcherright.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setTargetPosition(0);
        Turret.setDirection(DcMotor.Direction.REVERSE);
        launcherleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Last_Position_of_X = false;
        Last_Position_of_B = false;
        Last_Position_of_A = false;
        Last_Position_of_Y = false;
        intakeOn = true;
        Lifter.setDirection(Servo.Direction.REVERSE);
        Lifter.setPosition(-0.1);
        rightstopper.setPosition(0);
        rightstopper.setDirection(Servo.Direction.FORWARD);
        leftstopper.setPosition(1);
        leftstopper.setDirection(Servo.Direction.REVERSE);
        leftleg.setPosition(0.5);
        rightleg.setPosition(0.5);
        leftleg.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        ((DcMotorEx) launcherleft).setVelocityPIDFCoefficients(10, 0.0001, 0.0001, 13.5);
        ((DcMotorEx) launcherright).setVelocityPIDFCoefficients(10, 0.0001, 0.0001, 13.5);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            while (opModeIsActive()) {


                telemetryAprilTag();
                telemetry.update();


                sleep(20);
                axial = -gamepad1.left_stick_y;
                lateral = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                frontLeftPower = axial + lateral + yaw;
                frontRightPower = (axial - lateral) - yaw;
                backLeftPower = (axial - lateral) + yaw;
                backRightPower = (axial + lateral) - yaw;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                frontLeftPower = frontLeftPower / 2;
                frontRightPower = frontRightPower / 2;
                backLeftPower = backLeftPower / 2;
                backRightPower = backRightPower / 2;
                // Send calculated power to wheels.
                LeftFront.setPower(frontLeftPower);
                RightFront.setPower(frontRightPower);
                LeftRear.setPower(backLeftPower);
                RightRear.setPower(backRightPower);
                // LEG
                // INTAKE
                if (Last_Position_of_B == false && gamepad1.b == true) {
                    if (Intake.getPower() == 0.8) {
                        Intake.setPower(0);
                    } else {
                        Intake.setPower(0.8);
                    }
                }
                Last_Position_of_B = gamepad1.b;
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    leftleg.setPosition(-1);
                    rightleg.setPosition(-1);
                } else {
                    leftleg.setPosition(0.5);
                    rightleg.setPosition(0.5);
                }
                // LIFTER
                if (gamepad1.a) {
                    rightstopper.setPosition(0.25);
                    leftstopper.setPosition(0.25);
                    if (Intake.getPower() > 0) {
                        Intake.setPower(0);
                        sleep(200);
                        intakeOn = true;
                    } else {
                        intakeOn = false;
                    }
                    Lifter.setPosition(0.055);
                    sleep(400);
                    rightstopper.setPosition(0);
                    leftstopper.setPosition(0);
                    Lifter.setPosition(-0.1);
                    sleep(300);
                    if (intakeOn == true) {
                        Intake.setPower(0.8);
                    }
                    sleep(300);
                } else {
                    Lifter.setPosition(-0.1);
                }
                // Show the elapsed game time and wheel power.
                telemetry.addLine("Left Launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherleft).getVelocity(), 6, 1));
                telemetry.addLine("Right  Launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherright).getVelocity(), 6, 1));
                telemetry.update();
            }
        }
    }

    /*
     This Function Completely Handles AutoTurret Tracking and AprilTag Detection
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list containing the latest detections, which may be stale.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("#AprilTagsDetected", JavaUtil.listLength(myAprilTagDetections));
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("");
                telemetry.addLine("Turret" + JavaUtil.formatNumber(Turret.getCurrentPosition(), 6, 1));
                telemetry.addLine("");
                telemetry.addLine("====(ID" + myAprilTagDetection.id + ")" + myAprilTagDetection.metadata.name);
                telemetry.addLine("");
                telemetry.addLine("Bearing" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1));
                telemetry.addLine("");
                telemetry.addLine("Range" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range * 0.787402, 6, 1));
                telemetry.addLine("Left launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherleft).getVelocity(), 6, 1));
                telemetry.addLine("Right Launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherright).getVelocity(), 6, 1));
                telemetry.update();
            } else {
                telemetry.addLine("Turret" + JavaUtil.formatNumber(Turret.getCurrentPosition(), 6, 1));
                telemetry.addLine("====(ID" + myAprilTagDetection.id + ") Unknown");
                telemetry.update();
                telemetry.addLine("Left Launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherleft).getVelocity(), 6, 1));
                telemetry.update();
                telemetry.addLine("Right  Launcher" + JavaUtil.formatNumber(((DcMotorEx) launcherright).getVelocity(), 6, 1));
                telemetry.update();
            }

            // Turret Tracking Movement
            if (myAprilTagDetection.id == 24) {
                if (myAprilTagDetection.ftcPose.bearing > 1) {
                    Turret.setPower(-0.4);
                    sleep(20);
                    Turret.setPower(0);
                } else if (myAprilTagDetection.ftcPose.bearing < -1) {
                    Turret.setPower(0.4);
                    sleep(20);
                    Turret.setPower(0);
                } else if (Turret.getCurrentPosition() < -250) {
                    Turret.setPower(0.4);
                    sleep(20);
                    Turret.setPower(0);
                } else if (Turret.getCurrentPosition() > 250) {
                    Turret.setPower(-0.4);
                    sleep(20);
                    Turret.setPower(0);
                }
                if (false) {
                    // 7
                } else if (myAprilTagDetection.ftcPose.range >= 110 && myAprilTagDetection.ftcPose.range <= 150) {
                    // Far shot
                    ((DcMotorEx) launcherleft).setVelocity(1650);
                    ((DcMotorEx) launcherright).setVelocity(1650);
                } else if (myAprilTagDetection.ftcPose.range >= 70 && myAprilTagDetection.ftcPose.range <= 109) {
                    // Mid
                    ((DcMotorEx) launcherleft).setVelocity(1400);
                    ((DcMotorEx) launcherright).setVelocity(1400);
                } else if (myAprilTagDetection.ftcPose.range >= 20 && myAprilTagDetection.ftcPose.range <= 69) {
                    // Short
                    ((DcMotorEx) launcherleft).setVelocity(1350);
                    ((DcMotorEx) launcherright).setVelocity(1350);
                } else if (false) {
                    // 3
                    launcherleft.setPower(0.6);
                    launcherright.setPower(0.6);
                } else if (false) {
                    // 2
                    launcherleft.setPower(0.5);
                    launcherright.setPower(0.5);
                } else if (false) {
                    // 1
                    launcherleft.setPower(0.5);
                    launcherright.setPower(0.5);
                } else if (false) {
                    launcherleft.setPower(0.4);
                    launcherright.setPower(0.4);
                }
                //BLINKING CODE
                if (myAprilTagDetection.ftcPose.bearing > -5 && myAprilTagDetection.ftcPose.bearing < 5) {
                    Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                    sleep(100);
                    Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else {
                    Blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

            }
            telemetry.update();
        }
    }
}