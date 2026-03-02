package org.firstinspires.ftc.teamcode.autopaths.FTCDECODE2026.FarStarts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.ServoStopper;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueFarStartOneBall")
public class BlueFarStartOneBall extends LinearOpMode{
    intakeMotor intakemotor;
    flyWheelMotor flyWheel;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    ServoStopper servoStop;
    ServoStopper servoStop2;
    DcMotorEx turret;

    int counter;

    @Override
    public void runOpMode() {
        //CREATES STARTING POSE
        Pose2d beginPose = new Pose2d(new Vector2d(60, -14), Math.toRadians(180));

        //INIT PHASE -HARDWARE BELONGS HERE
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // ---------- HARDWARE ----------
        initialize();
        corrections();

        counter = 0;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(spinFar())
                .splineToLinearHeading( new Pose2d(50,-12,Math.toRadians(200)), Math.toRadians(205))
                .stopAndAdd(TriLift())
                .stopAndAdd(intakemotor.setpowerOn())
                .splineTo(new Vector2d(30,-30), 3 * Math.PI / 2)
                .lineToY(-60)
                .lineToY(-55)
                .splineToLinearHeading( new Pose2d(50,-12,Math.toRadians(200)), Math.toRadians(205))
                .stopAndAdd(TriLift())
                .stopAndAdd(intakemotor.setpowerOff())
                .build();

        Actions.runBlocking(new SequentialAction((path)));

        while(opModeIsActive())
            telemetry.addData("Looped Counter:", counter++);
    }

    //ACTION METHODS
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

    private Action spinDown() {
        return new ParallelAction(
                flyWheel.spinDown(),
                flyWheel2.spinDown()
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

    //SIMPLIFY METHODS
    private void initialize () {
        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));

        servoStop = new ServoStopper(hardwareMap.get(Servo.class, "left stopper"), 0.75, 1);
        servoStop2 = new ServoStopper(hardwareMap.get(Servo.class, "right stopper"), 0.3, 0);

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
}