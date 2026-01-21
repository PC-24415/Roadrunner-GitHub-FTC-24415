package org.firstinspires.ftc.teamcode.autopaths;

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

@Autonomous(name = "UnitTest")
public class UnitTest extends LinearOpMode {

    intakeMotor intakemotor;
    flyWheelMotor flyWheel ;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;
    ServoStopper servoStop;

    //TURRET TEST
    DcMotor Turret;
    double counter = 0;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(new Vector2d(-11, 23), Math.toRadians(90));

        // INIT PHASE — hardware belongs here
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //ACTION MECAHNISMS
        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));
        servoStop = new ServoStopper(hardwareMap.get(Servo.class, "stopper"));

        //NON-ACTION MECHANISMS
        Turret = hardwareMap.get(DcMotor.class, "Turret");

        //CORRECTIONS
        flyWheel.ReverseDirection();
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        Action LiftStopUp = new ParallelAction(
                servoLift.LiftUpCycle(),
                servoStop.StopDown()
        );

        Action LiftStopDown = new ParallelAction(
                servoLift.LiftDownCycle(),
                servoStop.StopUp()
        );

        Action shootClose =
                new SequentialAction(
                        flyWheel.spinUpClose(),
                        flyWheel2.spinUpClose(),
                        intakemotor.setpowerOn(),
                        new SleepAction(1),
                        LiftStopUp,
                        new SleepAction(0.2),
                        LiftStopDown,
                        LiftStopUp,
                        new SleepAction(0.2),
                        LiftStopDown,
                        LiftStopUp,
                        new SleepAction(0.2),
                        LiftStopDown,
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
                .stopAndAdd(intakemotor.setpowerOn())
                .waitSeconds(120)
                .stopAndAdd(intakemotor.setpowerOff())
                .waitSeconds(0.00001)
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(path);
        }

        while(opModeIsActive()){
            counter += 1;
            Turret.setPower(counter / 99999999.9);
            telemetry.addData("Counter ", counter);
            telemetry.addData("Turret Power", Turret.getPower());
            telemetry.update();
        }
    }
}