package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CLAW {
    private Servo Claw;
    public CLAW(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class, "Claw");
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(0.8);
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(0.2);
            return false;
        }
    }
    public Action openClaw() {
        return new OpenClaw();
    }
}
