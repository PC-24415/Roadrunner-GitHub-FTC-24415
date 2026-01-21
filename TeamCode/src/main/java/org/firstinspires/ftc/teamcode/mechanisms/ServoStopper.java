package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoStopper {
    Servo stop;

    public ServoStopper (Servo s){

        stop = s;
        stop.setDirection(Servo.Direction.REVERSE);
        stop.setPosition(0);
    }

    public class StopDown implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stop.setPosition(0.3);
                timer.reset();
                initialized = true;
            }

            packet.put("Servo pos", stop.getPosition());

            // keep running for 0.6 seconds
            return timer.seconds() < 0.6;
        }
    }

    public class StopUp implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stop.setPosition(0.0);
                timer.reset();
                initialized = true;
            }
            return timer.seconds() < 0.6;
        }
    }

    public Action StopDown(){return new ServoStopper.StopDown();}

    public Action StopUp(){return new ServoStopper.StopUp();}
}
