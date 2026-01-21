package org.firstinspires.ftc.teamcode.autopaths.FTCDECODE2026.testingcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ServoLifter;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;

@Autonomous(name = "OpModeTesting")
public class AutoTestingOpMode extends OpMode {

    //DRIVE AND PATH INFO
    MecanumDrive drive;
    Pose2d beginPose;

    //MECHANISMS DESIGNED TO IMPLEMENT ROADRUNNER ACTION SYSTEM
    intakeMotor intakemotor;
    flyWheelMotor flyWheel ;
    flyWheelMotor flyWheel2;
    ServoLifter servoLift;

    //LOGIC
    int i = 0;

    @Override
    public void init() {
        beginPose = new Pose2d(new Vector2d(-11, 23), Math.toRadians(90));

        // INIT PHASE — HARDWARE BELONGS HERE
        drive = new MecanumDrive(hardwareMap, beginPose);

        intakemotor = new intakeMotor(3.0, hardwareMap.get(DcMotor.class, "Intake"));
        flyWheel = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher right"));
        flyWheel2 = new flyWheelMotor(1.0, hardwareMap.get(DcMotorEx.class, "launcher left"));
        servoLift = new ServoLifter(1.0, hardwareMap.get(Servo.class, "Lifter"));

        //CORRECTIONS
        flyWheel.ReverseDirection();
        flyWheelMotor.RunFlyWheelUsingEncoders(flyWheel, flyWheel2);

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

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    public void start(){
        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(intakemotor.setpowerOn())
                .waitSeconds(120)
                .stopAndAdd(intakemotor.setpowerOff())
                .waitSeconds(0.00001)
                .build();
    }

    @Override
    public void loop() {
        i += 1;
        telemetry.addLine("THIS IS LOOPING CODE,");
        telemetry.addData("NUMBER OF LOOPS:", i);
        telemetry.update();
    }
}