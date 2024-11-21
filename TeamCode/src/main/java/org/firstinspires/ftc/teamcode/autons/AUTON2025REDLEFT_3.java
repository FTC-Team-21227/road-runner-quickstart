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
    public class ARM1 {
        private DcMotorEx arm1;

        public ARM1(HardwareMap hardwareMap) {
            arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
            arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

        public class LiftFloorUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 470) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftFloorUp() {
            return new AUTON2025REDLEFT_3.ARM1.LiftFloorUp();
        }



        public class LiftRungDown implements Action {
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
                if (pos < 2336) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftRungDown() {
            return new AUTON2025REDLEFT_3.ARM1.LiftRungDown();
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
                if (pos < 10232) {
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
                if (pos < 3953) {
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
                if (pos < 3540) {
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
                if (pos > 100) {
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
    public class INTAKE_ANGLE {
        private Servo Intake_Angle;

        public INTAKE_ANGLE(HardwareMap hardwareMap) {
            Intake_Angle = hardwareMap.get(Servo.class, "Intake_Angle");
        }

        public class RotatePosition0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake_Angle.setPosition(0.65);
                return false;
            }
        }
        public Action RotatePosition0() {
            return new RotatePosition0();
        }

        public class RotatePosition1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Intake_Angle.setPosition(0.98);
                return false;
            }
        }
        public Action RotatePosition1() {
            return new RotatePosition1();
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


        public class LiftRungDown implements Action {
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
                if (pos < 4700) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftRungDown() {
            return new AUTON2025REDLEFT_3.ARM2.LiftRungDown();
        }


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
                if (pos < 7510) {
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
                if (pos < 5158) {
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
                if (pos < 7020) {
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
        public class LiftFloorUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
                //packet.put("liftPos", pos);
                telemetry.update();
                if (pos > 7200) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftFloorUp(){
            return new AUTON2025REDLEFT_3.ARM2.LiftFloorUp();
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
//        Hook = hardwareMap.get(Servo.class, "Hook");
//        Claw = hardwareMap.get(Servo.class, "Claw");

        AUTON2025REDLEFT_3.INTAKE_ANGLE intake_angle = new AUTON2025REDLEFT_3.INTAKE_ANGLE(hardwareMap);

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)//go forward
                .lineToX(13);
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(13, 115))
//                .waitSeconds(2)
//                .turn(Math.toRadians(225))//face basket
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(115, 115))
//                .waitSeconds(2)
//
//                .turn(Math.toRadians(235)) //get second sample
//                .waitSeconds(2)
//                .turn(Math.toRadians(125)); //face basket
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, Math.toRadians(0))) //back up to hang specimen
                .lineToX(5);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, Math.toRadians(0)))
                .strafeTo(new Vector2d(13, 115));

        Action trajectoryActionCloseOut = tab1.fresh()
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
        claw.closeClaw();
        intake_angle.RotatePosition0();

        waitForStart();

        //if (isStopRequested()) return;

        Action firstTrajectory;
        Action secondTrajectory;
        Action thirdTrajectory;
        //if (startPosition == 1) {
        firstTrajectory = tab1.build();
        secondTrajectory = tab2.build();
        thirdTrajectory = tab3.build();
        //}
//        } else if (startPosition == 2) {
//            firstTrajectory = tab2.build();
//        } else {
//            firstTrajectory = tab3.build();
//        }

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                firstTrajectory
                        ),
                        new ParallelAction(
                                arm1.liftRungDown(),
                                arm2.liftRungDown(),
                                secondTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.liftFloorUp(),
                                arm2.liftFloorUp(),
                                thirdTrajectory
                        ),
                        new ParallelAction(
                            arm2.liftDown(),
                            arm1.liftDown()
                        )
                        //         trajectoryActionCloseOut
                )
        );
    }
}
