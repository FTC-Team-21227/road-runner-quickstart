package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ARM2 {
    private DcMotorEx arm2;

    public ARM2(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2"); //ARM2
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    //public

    public class LiftBucketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 7510) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftBucketUp() {
        return new LiftBucketUp();
    }
    public class LiftRungUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 6857) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }


    public class LiftLowBasketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos > 3340) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftLowBasketUp() {return new ARM2.LiftLowBasketUp();}



    public Action liftRungUp() {
        return new LiftRungUp();
    }
    public class LiftWallUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 3140) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftWallUp() {
        return new LiftWallUp();
    }
    public class HookSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 6732) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action hookSpecimen() {
        return new HookSpecimen();
    }
    public class LiftFloorUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos > 6593) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftFloorUp(){
        return new LiftFloorUp();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }
            double pos = arm2.getCurrentPosition();
            if (pos > 200) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown(){
        return new LiftDown();
    }
}