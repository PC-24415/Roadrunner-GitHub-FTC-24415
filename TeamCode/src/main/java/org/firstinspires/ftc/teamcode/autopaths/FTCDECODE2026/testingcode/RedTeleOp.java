package org.firstinspires.ftc.teamcode.autopaths.FTCDECODE2026.testingcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "RedTeleOp")
@Disabled
public class RedTeleOp extends OpMode {

    private boolean Run = false;

    private FtcDashboard dash = FtcDashboard.getInstance();

    //MECHANISMS
    intakeMotor intakemotor;
    flyWheelMotor flyWheel ;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    DcMotor Turret;
    //CAMERA
    VisionPortal visionPortal = null;
    //CAMERA EXPOSURE
    private int myExposure, minExposure, maxExposure;
    private int myGain, minGain, maxGain;
    private boolean thisExpUp, thisExpDn, thisGainUp, thisGainDn;
    private boolean lastExpUp, lastExpDn, lastGainUp, lastGainDn;
    //APRIL TAG DETECTION
    AprilTagProcessor aprilTag;
    //MOTORS FOR EACH WHEEL
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor rightFront;

    //PID CONTROL FOR TURRET
    private PDController pid;

    private static final double kP = 0.0005;
    private static final double kD = 0.0001;
    private static final double acceptableTurretError = 10.0;

    //INITIALIZATION FOR EACH MOTOR, MECHANISM, CAMERA
    @Override
    public void init() {
        //MECHANISM INITIALIZATION
        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotor.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotor.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");

        flyWheel.ReverseDirection();

        //DRIVE MOTOR INITIALIZATION
        leftFront = hardwareMap.get(DcMotorEx.class, "Left Front");
        leftBack = hardwareMap.get(DcMotorEx.class, "Left Rear");
        rightBack = hardwareMap.get(DcMotorEx.class, "Right Rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "Right Front");

        //CAMERA INITIALIZATION
        initAprilTag();
        getCameraSetting();

        telemetry.addData("Camera", "Ready — press START");

        //READY TO RUN OpMode
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        dash.sendTelemetryPacket(new TelemetryPacket());

        //APRIL TAG DETECTIONS
        List<AprilTagDetection> detections = aprilTag.getDetections();

        //RUN LOGIC
        Run = true;

        //DRIVETRAIN MOVEMENT
        leftFront.setPower(-this.gamepad1.left_stick_x);
        leftBack.setPower(-this.gamepad1.left_stick_x);
        rightBack.setPower(-this.gamepad1.left_stick_x);
        rightFront.setPower(-this.gamepad1.left_stick_x);

        //APRILTAG LOGIC
        if (!detections.isEmpty()) {
            AprilTagDetection det = detections.get(0);
            double tagX = det.center.x;
            double frameCenter = 640.0 / 2.0;
            double errorX = tagX - frameCenter;

            telemetry.addData("Tag", "Detected ID %d", det.id);
            telemetry.addData("X Error", "%.1f", errorX);

            if (Math.abs(errorX) > acceptableTurretError) {
                pid.setSetPoint(0);
                double turretPow = pid.calculate(errorX);
                Turret.setPower(turretPow);
                telemetry.addData("Turret Power", turretPow);
            } else {
                Turret.setPower(0);
            }
        } else {
            telemetry.addData("Tag", "None detected");
            Turret.setPower(0);
        }

        telemetry.addLine("Running TeleOp");
        telemetry.update();
    }

    @Override
    public void stop() {
        //ENDS RUN LOGIC
        Run = false;

        //CUTS MOTOR POWER
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        //ENDS CAMERA VISION
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void getCameraSetting() {
        if (visionPortal == null) return;
        while (Run && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting to open...");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
    }
}
