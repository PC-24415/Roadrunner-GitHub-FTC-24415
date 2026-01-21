package org.firstinspires.ftc.teamcode.autopaths;


import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "TestPath")
public class TestPath extends LinearOpMode{

    intakeMotor intakemotor;
    flyWheelMotor flyWheel ;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;

    @Override
    public void runOpMode() throws InterruptedException{
        //Create Starting Pose
        Pose2d beginPose = new Pose2d(new Vector2d(-11, 23), Math.toRadians(90));

        // INIT PHASE — hardware belongs here
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));

        flyWheel.ReverseDirection();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action shootClose =
                new SequentialAction(
                        flyWheel.spinUpClose(),
                        flyWheel2.spinUpClose(),
                        new SleepAction(2),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        new SleepAction(0.5),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        new SleepAction(0.5),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        flyWheel.spinDown(),
                        flyWheel2.spinDown()
                );

        Action shootFar =
                new SequentialAction(
                        flyWheel.spinUpFar(),
                        flyWheel2.spinUpFar(),
                        new SleepAction(2),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        new SleepAction(0.5),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        new SleepAction(0.5),
                        intakemotor.setpowerOn(),
                        new SleepAction(0.25),
                        intakemotor.setpowerOff(),
                        servoLift.LiftUpCycle(),
                        new SleepAction(0.5),
                        servoLift.LiftDownCycle(),
                        flyWheel.spinDown(),
                        flyWheel2.spinDown()
                );

        Action path = drive.actionBuilder(beginPose)
                .lineToY(14)
                .stopAndAdd(shootClose)
                .waitSeconds(1)
                .lineToY(23)
                .stopAndAdd(shootFar)
                .waitSeconds(0.1)
                .build();

        Actions.runBlocking(new SequentialAction((path)));
    }
}