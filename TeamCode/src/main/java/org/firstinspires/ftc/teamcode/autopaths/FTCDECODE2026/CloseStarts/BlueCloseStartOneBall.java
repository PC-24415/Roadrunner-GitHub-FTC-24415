package org.firstinspires.ftc.teamcode.autopaths.FTCDECODE2026.CloseStarts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueCloseStartOneBall")
public class BlueCloseStartOneBall extends LinearOpMode{

    //Devices (Motors/Servos/ect)
    intakeMotor intakemotor;
    flyWheelMotor flyWheel ;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;

    @Override
    public void runOpMode() throws InterruptedException{
        //Create Starting Pose
        Pose2d beginPose = new Pose2d(new Vector2d(-50, -48), Math.toRadians(225));

        //Create Roadrunner drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotor.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotor.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));

        flyWheel.ReverseDirection();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action shootClose =
                new SequentialAction(
                        flyWheel.spinUpClose(),
                        flyWheel2.spinUpClose(),
                        new SleepAction(1),
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

        Action shootFar = new SequentialAction(
                flyWheel.spinUpFar(),
                flyWheel2.spinUpFar(),
                new SleepAction(1),
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
                .lineToY(-14)
                //Shoot
                .stopAndAdd(shootClose)
                //Intake ON
                .stopAndAdd(intakemotor.setpowerOn())
                .splineTo(new Vector2d(-12,-49), Math.toRadians(270))
                //Intake Off
                .stopAndAdd(intakemotor.setpowerOff())
                .splineToLinearHeading(new Pose2d(-24,-24,Math.toRadians(135 + 90)), Math.toRadians(135 + 90))
                //Shoot
                .stopAndAdd(shootClose)
                .build();

        Actions.runBlocking(new SequentialAction((path)));
    }
}