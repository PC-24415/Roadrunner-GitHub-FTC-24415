package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class TiberiusServo {
    Servo servo;
    double target;
    double def;

    public TiberiusServo(Servo s, double t, double d) {
        servo = s;
        target = t;
        def = d;
    }

    public class setTarget implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                servo.setPosition(target);
                initialized = true;
            }
            return false;
        }
    }
    public Action SetTarget() { return new TiberiusServo.setTarget(); }

    public class setDef implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                servo.setPosition(def);
                initialized = true;
            }
            return false;
        }
    }

    public Action SetDef() { return new TiberiusServo.setDef(); }

    public void reverseServoDirection() {
        this.servo.setDirection(Servo.Direction.REVERSE);
    }
}