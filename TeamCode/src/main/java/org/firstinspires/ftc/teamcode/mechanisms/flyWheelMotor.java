package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class flyWheelMotor {

    double targetPower;

    DcMotorEx flyWheel;

    public flyWheelMotor(double targetPower, DcMotorEx f) {
        this.targetPower = targetPower;
        flyWheel = f;
    }

    public static void RunFlyWheelUsingEncoders(flyWheelMotor o, flyWheelMotor t)
    {
        o.flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        o.flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t.flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        t.flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ReverseDirection() {

        this.flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class SpinUpClose implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flyWheel.setVelocity(320)       ;//flyWheel.setPower(0.6);
                initialized = true;
            }

            packet.put("Fly Wheel Power", flyWheel.getPower());
            return false;
        }
    }

    public class SpunUpFar implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flyWheel.setVelocity(320);       //flyWheel.setPower(0.83);
                initialized = true;
            }

            packet.put("Fly Wheel Power", flyWheel.getPower());
            return false;
        }
    }

    public class SpinDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                flyWheel.setVelocity(0);     //flyWheel.setPower(0.0);
                initialized = true;
            }

            packet.put("Fly Wheel Power", flyWheel.getPower());
            return false;
        }
    }

    public Action spinUpClose() {
        return new SpinUpClose();
    }

    public Action spinUpFar() {
        return new SpunUpFar();
    }

    public Action spinDown(){
        return new SpinDown();
    }
}
