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
import org.firstinspires.ftc.teamcode.mechanisms.TiberiusMotor;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.TiberiusMotorEncoder;
import org.firstinspires.ftc.teamcode.mechanisms.TiberiusServo;

@Autonomous(name = "TiberiusPath")
public class TiberiusCode extends OpMode {

    MecanumDrive drive;
    Pose2d beginPose;

    TiberiusMotor intake;
    TiberiusMotorEncoder flywheel1;
    TiberiusMotorEncoder flywheel2;
    TiberiusServo lifter;
    TiberiusServo stopper1;
    TiberiusServo stopper2;

    @Override
    public void init() {

        intake = new TiberiusMotor(hardwareMap.get(DcMotor.class, "Intake"), 0.7, 0);
        flywheel1 = new TiberiusMotorEncoder(hardwareMap.get(DcMotorEx.class, "launcher right"), 1600, 1350);
        flywheel2 = new TiberiusMotorEncoder(hardwareMap.get(DcMotorEx.class, "launcher left"), 1600, 1350);

        lifter = new TiberiusServo(hardwareMap.get(Servo.class, "Lifter"), 0.06,-0.1);
        stopper1 = new TiberiusServo(hardwareMap.get(Servo.class, "left stopper"), 0.75, 1);
        stopper2 = new TiberiusServo(hardwareMap.get(Servo.class, "right stopper"), 0.3, 0);

        beginPose = new Pose2d(new Vector2d(-50, -48), Math.toRadians(225));
        drive = new MecanumDrive(hardwareMap, beginPose);

        flywheel1.reverserMotorDirection();
        lifter.reverseServoDirection();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addLine("OpMode has started");
        telemetry.update();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(liftThree())
                .build();

        Actions.runBlocking(path);
    }

    @Override
    public void loop() {
        telemetry.update();
    }

    private Action liftThree() {
        return new SequentialAction(
                intake.SetTarget(),
                flywheel1.SetDef(),
                flywheel2.SetDef(),
                new SleepAction(3),
                intake.SetDef(),
                lifter.SetTarget(),
                new SleepAction(1),
                lifter.SetDef(),
                intake.SetTarget(),
                new SleepAction(0.5),
                intake.SetDef(),
                lifter.SetTarget(),
                new SleepAction(1),
                lifter.SetDef(),
                intake.SetTarget(),
                new SleepAction(0.5),
                intake.SetDef(),
                lifter.SetTarget(),
                new SleepAction(1),
                lifter.SetDef(),
                intake.SetTarget(),
                new SleepAction(3),
                intake.SetDef()
                );
    }
}