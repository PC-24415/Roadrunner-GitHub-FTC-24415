package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TiberiusMotorEncoder {
    DcMotorEx motor;
    double velocity;
    double def;

    public TiberiusMotorEncoder(DcMotorEx m, double v, double d) {
        motor = m;
        velocity = v;
        def = d;
    }

    public class setVelocity implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setVelocity(velocity);
                initialized = true;
            }
            return false;
        }
    }

    public Action SetVelocity() {
        return new TiberiusMotorEncoder.setVelocity();
    }

    public class setDef implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setVelocity(def);
                initialized = true;
            }
            return false;
        }
    }

    public Action SetDef() {
        return new TiberiusMotorEncoder.setDef();
    }

    public void reverserMotorDirection() {
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}