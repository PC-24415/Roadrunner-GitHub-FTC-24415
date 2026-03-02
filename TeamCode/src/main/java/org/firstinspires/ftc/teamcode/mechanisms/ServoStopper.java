package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoStopper {
    Servo stop;
    double target;
    double def;

    public ServoStopper (Servo s, double t, double d){

        stop = s;
        stop.setDirection(Servo.Direction.FORWARD);
        target = t;
        def = d;
        stop.setPosition(def);
    }

    public static void ReverseDirection(ServoStopper s)
    {
        s.stop.setDirection(Servo.Direction.REVERSE);
    }

    public class StopDown implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stop.setPosition(target);
                timer.reset();
                initialized = true;
            }

            packet.put("Servo pos", stop.getPosition());

            return false;
        }
    }

    public class StopUp implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stop.setPosition(def);
                timer.reset();
                initialized = true;
            }
            return false;
        }
    }

    public Action StopDown(){return new ServoStopper.StopDown();}

    public Action StopUp(){return new ServoStopper.StopUp();}
}
