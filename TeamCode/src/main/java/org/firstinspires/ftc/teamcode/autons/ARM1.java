package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ARM1 {
    private DcMotorEx arm1;

    public ARM1(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public class hookSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(-1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos < 5409) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action hookSpecimen() {
        return new hookSpecimen();
    }
    public class LiftBucketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos < 10413) {
                return true;
            } else {
                arm1.setPower(0);
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
                arm1.setPower(1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos < 4258) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftRungUp() {
        return new LiftRungUp();
    }
    public class LiftWallUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos < 1351) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftWallUp() {
        return new LiftWallUp();
    }


    public class LiftLowBasketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos < 6400) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftLowBasketUp() {return new LiftLowBasketUp();}


    public class LiftFloorDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(-1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos > 0) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action LiftFloorDown() {return new LiftFloorDown();}
    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm1.setPower(-1);
                initialized = true;
            }

            double pos = arm1.getCurrentPosition();
            if (pos > 200) { //change if the lift presses down too hard
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown(){
        return new LiftDown();
    }
}