package org.firstinspires.ftc.teamcode.autopaths;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.ServoStopper;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name = "TestPath")
public class TestPath extends LinearOpMode {
    intakeMotor intakemotor;
    flyWheelMotor flyWheel;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    ServoStopper servoStop;
    ServoStopper servoStop2;

    @Override
    public void runOpMode() {
        //CREATES STARTING POSE
        Pose2d beginPose = new Pose2d(new Vector2d(-11, 23), Math.toRadians(90));

        //INIT PHASE -HARDWARE BELONGS HERE
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // ---------- HARDWARE ----------
        initialize();

        corrections();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .lineToY(14)
                .stopAndAdd(shootClose())
                .waitSeconds(1)
                .lineToY(23)
                .stopAndAdd(shootClose())
                .waitSeconds(0.1)
                .build();

        Actions.runBlocking(path);
    }

    //ACTION METHODS
    private Action liftStopUp() {
        return new ParallelAction(
                intakemotor.setpowerOff(),
                servoLift.LiftUpCycle(),
                servoStop.StopDown(),
                servoStop2.StopDown()
        );
    }

    private Action liftStopDown() {
        return new ParallelAction(
                servoLift.LiftDownCycle(),
                servoStop.StopUp(),
                servoStop2.StopUp(),
                intakemotor.setpowerOn()
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

    private Action shootClose() {
        return new SequentialAction(
                spinClose(),
                new SleepAction(0.75),
                TriLift(),
                spinDown()
        );
    }

    private Action shootFar() {
        return new SequentialAction(
                spinFar(),
                new SleepAction(1),
                TriLift(),
                spinDown()
        );
    }

    private Action TriLift() {
        return new SequentialAction(
                liftStopUp(),
                new SleepAction(0.2),
                liftStopDown(),
                new SleepAction(0.2),

                liftStopUp(),
                new SleepAction(0.2),
                liftStopDown(),
                new SleepAction(0.2),

                liftStopUp(),
                new SleepAction(0.2),
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