package org.firstinspires.ftc.teamcode.autopaths.PostFTC2026DecodeCode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.ServoStopper;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;

@Autonomous(name = "Elijah Path")
public class ElijahCode extends OpMode {
    intakeMotor intakemotor;
    flyWheelMotor flyWheel;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    ServoStopper servoStop;
    ServoStopper servoStop2;

    MecanumDrive drive;
    Pose2d beginPose;

    @Override
    public void init() {
        //CREATES STARTING POSE
        beginPose = new Pose2d(new Vector2d(-50, -48), Math.toRadians(225));

        //INIT PHASE -HARDWARE BELONGS HERE
        drive = new MecanumDrive(hardwareMap, beginPose);

        initialize();
        corrections();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        telemetry.addLine("OpMode has started");
        telemetry.update();

        Action path = drive.actionBuilder(beginPose)
                .lineToY(-40)
                .turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(-31, -22),(Math.toRadians(0)))
                .splineTo(new Vector2d(-12, -30),(Math.toRadians(270)))
                .lineToY(-52)
                .lineToY(0)
                .turnTo(Math.toRadians(225))
                .waitSeconds(3)
                .splineTo(new Vector2d(11, -30),(Math.toRadians(270)))
                .lineToY(-52)
                .splineTo(new Vector2d(-12, 0),(Math.toRadians(225)))
                .waitSeconds(3)
                .build();

        Actions.runBlocking(path);
        telemetry.update();

    }

    @Override
    public void loop() {
        telemetry.addLine("This Is Looped Code");
        telemetry.update();
    }

    @Override
    public void stop(){
        telemetry.addLine("OpMode has finished");
        telemetry.update();
    }

    //ACTION METHODS
    private Action liftStopUp() {
        return new ParallelAction(
                //intakemotor.setpowerOff(),
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
                //intakemotor.setpowerOn()
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
                liftStopUp(),
                new SleepAction(0.8),
                liftStopDown(),
                new SleepAction(0.8),

                liftStopUp(),
                new SleepAction(0.8),
                liftStopDown(),
                new SleepAction(0.8),

                liftStopUp(),
                new SleepAction(0.8),
                liftStopDown()
        );
    }

    //SIMPLIFY METHODS
    private void initialize () {
        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));
        //FACING THE INTAKE THIS IS THE RIGHT SERVO
        servoStop = new ServoStopper(hardwareMap.get(Servo.class, "left stopper"), 0.6, 1);
        //FACING INTAKE THIS IS THE LEFT SERVO
        servoStop2 = new ServoStopper(hardwareMap.get(Servo.class, "right stopper"), 0.4, 0);
    }

    private void corrections(){
        flyWheel.ReverseDirection();
        flyWheelMotor.RunFlyWheelUsingEncoders(flyWheel, flyWheel2);
    }
}
