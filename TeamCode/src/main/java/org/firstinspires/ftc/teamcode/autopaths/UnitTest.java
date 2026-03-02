package org.firstinspires.ftc.teamcode.autopaths;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.ServoStopper;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name = "UnitTest")
public class UnitTest extends LinearOpMode {

    // -------- MECHANISMS --------
    intakeMotor intakemotor;
    flyWheelMotor flyWheel;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    ServoStopper servoStop;
    ServoStopper servoStop2;
    DcMotorEx turret;

    // -------- VISION --------
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;
    boolean USE_WEBCAM = true;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(new Vector2d(-11, 23), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        initialize();
        corrections();
        initAprilTag();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action path = new SequentialAction(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(spinFar())
                        .waitSeconds(1)
                        .stopAndAdd(TriLift())
                        .waitSeconds(1)
                        .build()
        );

        Actions.runBlocking(path);

        while (opModeIsActive()) {
            telemetry.addLine("Loop");
            telemetry.update();
//            // ----- Vision Tracking -----
//            List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
//
//            // Show how many detections we see
//            telemetry.addData("Total Detections", detections.size());
//
//            if (!detections.isEmpty()) {
//                for (AprilTagDetection tag : detections) {
//
//                    // Check if the tag has a valid pose
//                    if (tag.ftcPose == null) {
//                        telemetry.addData("Tag ID", tag.id);
//                        telemetry.addLine("No pose detected for this tag!");
//                        continue;
//                    }
//
//                    double bearing = tag.ftcPose.bearing;
//                    double range = tag.ftcPose.range;
//
//                    // Telemetry for the tag
//                    telemetry.addData("Tag ID", tag.id);
//                    telemetry.addData("Bearing", bearing);
//                    telemetry.addData("Range", range);
//
//                    // Turret tracking
//                    if (bearing > 1.0) {
//                        turret.setPower(-0.3);
//                        telemetry.addLine("Turret Power: -0.3 (turning right)");
//                    } else if (bearing < -1.0) {
//                        turret.setPower(0.3);
//                        telemetry.addLine("Turret Power: 0.3 (turning left)");
//                    } else {
//                        turret.setPower(0);
//                        telemetry.addLine("Turret Power: 0 (on target)");
//                    }
//
//                    // Turret limits
//                    if (turret.getCurrentPosition() > 250) {
//                        turret.setPower(-0.2);
//                        telemetry.addLine("Turret max limit reached, reversing");
//                    }
//                    if (turret.getCurrentPosition() < -250) {
//                        turret.setPower(0.2);
//                        telemetry.addLine("Turret min limit reached, reversing");
//                    }
//
//                    // Flywheel control
//                    if (range >= 110 && range <= 150) {
//                        flyWheel.spinTarget(1650);
//                        flyWheel2.spinTarget(1650);
//                        telemetry.addLine("Flywheel set for far range");
//                    } else if (range >= 70 && range <= 109) {
//                        flyWheel.spinTarget(1400);
//                        flyWheel2.spinTarget(1400);
//                        telemetry.addLine("Flywheel set for mid range");
//                    } else if (range >= 20 && range <= 69) {
//                        flyWheel.spinTarget(1350);
//                        flyWheel2.spinTarget(1350);
//                        telemetry.addLine("Flywheel set for short range");
//                    }
//
//                    // Only use the first valid tag
//                    break;
//                }
//            } else {
//                turret.setPower(0);
//                telemetry.addLine("No AprilTags detected!");
//            }
//
//            // Always show turret position
//            telemetry.addData("Turret Position", turret.getCurrentPosition());
//            telemetry.update();
        }
    }

    // ================= ACTIONS =================

    private Action liftStopUp() {
        return new ParallelAction(
                servoLift.LiftUpCycle(),
                servoStop.StopDown(),
                servoStop2.StopDown()
        );
    }

    private Action liftStopDown() {
        return new ParallelAction(
                servoLift.LiftDownCycle(),
                servoStop.StopUp(),
                servoStop2.StopUp()
        );
    }

    private Action spinClose() {
        return new ParallelAction(
                flyWheel.spinUpClose(),
                flyWheel2.spinUpClose()
        );
    }

    private Action spinFar() {
        return new ParallelAction(
                flyWheel.spinUpFar(),
                flyWheel2.spinUpFar()
        );
    }

    private Action TriLift() {
        return new SequentialAction(
                intakemotor.setpowerOff(),
                liftStopUp(),
                new SleepAction(0.5),
                intakemotor.setpowerOn(),
                liftStopDown(),
                new SleepAction(1),
                intakemotor.setpowerOff(),
                liftStopUp(),
                new SleepAction(0.5),
                intakemotor.setpowerOn(),
                liftStopDown(),
                new SleepAction(1),
                intakemotor.setpowerOff(),
                liftStopUp(),
                new SleepAction(0.5),
                liftStopDown(),
                intakemotor.setpowerOn()
        );
    }

    // ================= INIT =================

    private void initialize() {
        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));

        servoStop = new ServoStopper(hardwareMap.get(Servo.class, "left stopper"), 0.9, 1);
        servoStop2 = new ServoStopper(hardwareMap.get(Servo.class, "right stopper"), 0.25, 0);

        turret = hardwareMap.get(DcMotorEx.class, "Turret");
    }

    private void corrections(){
        flyWheel.ReverseDirection();
        flyWheelMotor.RunFlyWheelUsingEncoders(flyWheel, flyWheel2);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.setTargetPositionTolerance(1);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.2);
    }

    private void initAprilTag() {

        myAprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            portalBuilder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam 1"));
        }

        portalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = portalBuilder.build();
    }
}
