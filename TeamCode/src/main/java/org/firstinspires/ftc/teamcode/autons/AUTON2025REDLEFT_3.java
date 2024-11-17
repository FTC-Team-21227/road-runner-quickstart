package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AUTON2025REDLEFT_3")
public class AUTON2025REDLEFT_3 extends LinearOpMode{
    private Servo Hook;
    private Servo Claw;
    private Servo Intake_Angle;
    public class ARM1 {
        private DcMotorEx arm1;

        public ARM1(HardwareMap hardwareMap) {
            arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
            arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }
        public class LiftBucketUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 8508) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftBucketUp() {
            return new AUTON2025REDLEFT_3.ARM1.LiftBucketUp();
        }
        public class LiftRungUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 3140) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftRungUp() {
            return new AUTON2025REDLEFT_3.ARM1.LiftRungUp();
        }
        public class LiftWallUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 3140) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftWallUp() {
            return new AUTON2025REDLEFT_3.ARM1.LiftWallUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
                //packet.put("liftPos", pos);
                telemetry.update();
                if (pos > -100) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new AUTON2025REDLEFT_3.ARM1.LiftDown();
        }
    }
    public class CLAW {
        private Servo Claw;

        public CLAW(HardwareMap hardwareMap) {
            Claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    public class HOOK {
        private Servo Hook;

        public HOOK(HardwareMap hardwareMap) {
            Hook = hardwareMap.get(Servo.class, "Hook");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Hook.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Hook.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
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
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//              packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 6020) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftBucketUp() {
            return new AUTON2025REDLEFT_3.ARM2.LiftBucketUp();
        }
        public class LiftRungUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//              packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 3517) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftRungUp() {
            return new AUTON2025REDLEFT_3.ARM2.LiftRungUp();
        }
        public class LiftWallUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 3140) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftWallUp() {
            return new AUTON2025REDLEFT_3.ARM2.LiftWallUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
                //packet.put("liftPos", pos);
                telemetry.update();
                if (pos > 200) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new AUTON2025REDLEFT_3.ARM2.LiftDown();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AUTON2025REDLEFT_3.ARM1 arm1 = new AUTON2025REDLEFT_3.ARM1(hardwareMap);
        AUTON2025REDLEFT_3.ARM2 arm2 = new AUTON2025REDLEFT_3.ARM2(hardwareMap);

        AUTON2025REDLEFT_3.CLAW claw = new AUTON2025REDLEFT_3.CLAW(hardwareMap);
        AUTON2025REDLEFT_3.HOOK hook = new AUTON2025REDLEFT_3.HOOK(hardwareMap);
//        Hook = hardwareMap.get(Servo.class, "Hook");
//        Claw = hardwareMap.get(Servo.class, "Claw");

        Intake_Angle = hardwareMap.get(Servo.class,"Intake_Angle");

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(30)
                .waitSeconds(2)
                .strafeTo(new Vector2d(13, 115))
                .waitSeconds(2)
                .turn(Math.toRadians(225))//face basket
                .waitSeconds(2)
                .strafeTo(new Vector2d(115, 115))
                .waitSeconds(2)

                .turn(Math.toRadians(235)) //get second sample
                .waitSeconds(2)
                .turn(Math.toRadians(125)) //face basket
                ;
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)

//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", arm1.getCurrentPosition);
//            telemetry.update();
//        }
//
//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
        waitForStart();

        //if (isStopRequested()) return;

        Action firstTrajectory;
        //if (startPosition == 1) {
        firstTrajectory = tab1.build();
        //}
//        } else if (startPosition == 2) {
//            firstTrajectory = tab2.build();
//        } else {
//            firstTrajectory = tab3.build();
//        }

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                firstTrajectory
                        ),
//                        claw.openClaw(),
                        new ParallelAction(
                            arm2.liftDown(),
                            arm1.liftDown()
                        )
                        //         trajectoryActionCloseOut
                )
        );
    }
}
