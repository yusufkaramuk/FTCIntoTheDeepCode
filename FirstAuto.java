package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name = "FirstAuto")
public class FirstAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -60, Math.toRadians(90)));

        Servo KiskacServo = hardwareMap.get(Servo.class, "KiskacServo");
        DcMotorEx leftElevator = hardwareMap.get(DcMotorEx.class, "leftElevator");
        DcMotorEx rightElevator = hardwareMap.get(DcMotorEx.class, "rightElevator");

        leftElevator.setDirection(DcMotorEx.Direction.FORWARD);
        rightElevator.setDirection(DcMotorEx.Direction.REVERSE);

        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Motorları sıfırla ve hazır hale getir
        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setTargetPosition(0);
        rightElevator.setTargetPosition(0);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        int AsansorAskiMax = 1900;
        int AsansorAskiMin = 1000;
        int AsansorMin = 10;


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                        .strafeTo(new Vector2d(35, -36))
                        .strafeTo(new Vector2d(35, -12))
                        .strafeTo(new Vector2d(42, -12))
                        .strafeTo(new Vector2d(42, -51.63))
                        .splineToLinearHeading(new Pose2d(38.00, -55.5, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .strafeTo(new Vector2d(38, -58))
                        //kıskac
                        .stopAndAdd(new ServoAction(KiskacServo, 0.0))
                        .waitSeconds(1)
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, 1, AsansorAskiMax, this))
                        .strafeTo(new Vector2d(35.00, -51.05))
                        .splineToLinearHeading(new Pose2d(11.95, -41.56, Math.toRadians(125.00)), Math.toRadians(125.00))
                        .splineToLinearHeading(new Pose2d(4.27, -35, Math.toRadians(90.00)), Math.toRadians(270.00))
                        .strafeTo(new Vector2d(4.27, -28))
                        // Asansörü askı yüksekliğine indir
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, 1, AsansorAskiMin, this))
                        // Servo aç
                        .stopAndAdd(new ServoAction(KiskacServo, 1.0))
                        .strafeTo(new Vector2d(4.27, -36))



                        .splineToLinearHeading(new Pose2d(35.00, -36.00, Math.toRadians(90.00)), Math.toRadians(90)) // Geri geri giderek X:35 Y:-36 noktasına ulaşır
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, 0.7, AsansorMin, this))

                        .splineToLinearHeading(new Pose2d(38.00, -55.5, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .stopAndAdd(new ServoAction(KiskacServo, 0.0))
                        .waitSeconds(1)
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, 1, AsansorAskiMax, this))





                        //Human-Askı Yolu
                        .strafeTo(new Vector2d(35.00, -51.05))
                        .splineToLinearHeading(new Pose2d(11.95, -41.56, Math.toRadians(125.00)), Math.toRadians(125.00))
                        .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(90.00)), Math.toRadians(270.00))
                        .strafeTo(new Vector2d(0, -30))
                        // Asansörü askı yüksekliğine indir
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, 1, AsansorAskiMin, this))
                        // Servo aç
                        .stopAndAdd(new ServoAction(KiskacServo, 1.0))
                        .waitSeconds(0.5)

                        .strafeTo(new Vector2d(0, -36))



                        .build());
    }
}

// **ElevatorAction Sınıfı**
class ElevatorAction implements Action {
    private final DcMotorEx leftElevator, rightElevator;
    private final double power;
    private final int targetPosition;
    private final LinearOpMode opMode;

    public ElevatorAction(DcMotorEx leftElevator, DcMotorEx rightElevator, double power, int targetPosition, LinearOpMode opMode) {
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.power = power;
        this.targetPosition = targetPosition;
        this.opMode = opMode;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        leftElevator.setTargetPosition(targetPosition);
        rightElevator.setTargetPosition(targetPosition);

        leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftElevator.setPower(power);
        rightElevator.setPower(power);

        while ((leftElevator.isBusy() || rightElevator.isBusy()) && opMode.opModeIsActive()) {
            telemetryPacket.put("Left Elevator Position", leftElevator.getCurrentPosition());
            telemetryPacket.put("Right Elevator Position", rightElevator.getCurrentPosition());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        leftElevator.setPower(0);
        rightElevator.setPower(0);
        return false;
    }
}

// **ServoAction Sınıfı**
class ServoAction implements Action {
    private final Servo servo;
    private final double position;

    public ServoAction(Servo servo, double position) {
        this.servo = servo;
        this.position = position;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        servo.setPosition(position);
        telemetryPacket.put("Servo Position", position);
        return false;
    }
}
