package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "FirstAuto")
public class FirstAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -60, Math.toRadians(90)));

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
        int AsansorMax= 1500;
        int AsansorAski= 1055;
        int AsansorMin= 0;

        KiskacServo.setPosition(0.0); // Servo kapalı konum


        waitForStart();

        if (isStopRequested()) return;  // Eğer durdurulmuşsa çık

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        // **Servo Kodu** (!Servo Kapalı İken 0 Değerine Sahip!)
                        // .stopAndAdd(new ServoAction(KiskacServo, 0))

                        // Asansörü maksimum yüksekliğe çıkar
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, AsansorMax, 0.5))

                        // Hedef konuma hareket et
                        .splineTo(new Vector2d(0, -34), Math.toRadians(90.00))

                        // Asansörü askı yüksekliğine indir
                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, AsansorAski, 0.3))

                        // Servo aç
                        .stopAndAdd(new ServoAction(KiskacServo, 1.0)) // 1.0: Açık konum

                        .strafeTo(new Vector2d(0, -36))

                        .stopAndAdd(new ElevatorAction(leftElevator, rightElevator, AsansorMin, 0.5))

                        .strafeTo(new Vector2d(0, -37.31))



                                // **Hareket Planı**
                        /*.splineTo(new Vector2d(34.95, -34.42), Math.toRadians(90.00))
                        .splineTo(new Vector2d(35, -10.19), Math.toRadians(90.00))
                        .strafeTo(new Vector2d(47.17, -10.19))
                        .strafeTo(new Vector2d(47.17, -51.63))
                        .strafeTo(new Vector2d(47.17, -10.19))
                        .strafeTo(new Vector2d(57.17, -10.19))
                        .strafeTo(new Vector2d(57.17, -51.63))
                        .strafeTo(new Vector2d(57.17, -10.19))
                        .strafeTo(new Vector2d(61, -10.19))
                        .strafeTo(new Vector2d(61, -51.63))
                        .strafeTo(new Vector2d(61, -33.29))
                        .strafeTo(new Vector2d(0, -33.29))*/

                        .build());
    }
}

// **Asansör Motoru İçin Aksiyon Sınıfı**
class ElevatorAction implements Action {
    private final DcMotorEx leftElevator, rightElevator;
    private final int targetPosition;
    private final double power;

    public ElevatorAction(DcMotorEx left, DcMotorEx right, int position, double power) {
        this.leftElevator = left;
        this.rightElevator = right;
        this.targetPosition = position;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        leftElevator.setTargetPosition(targetPosition);
        rightElevator.setTargetPosition(targetPosition);

        leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftElevator.setPower(power);
        rightElevator.setPower(power);

        // **Asansör Hareketi Tamamlanana Kadar Bekle**
        while (leftElevator.isBusy() || rightElevator.isBusy()) {
            telemetryPacket.put("Left Elevator Position", leftElevator.getCurrentPosition());
            telemetryPacket.put("Right Elevator Position", rightElevator.getCurrentPosition());
        }

        // **Motorları Durdur**
        leftElevator.setPower(0);
        rightElevator.setPower(0);

        return false;
    }
}

// **Servo İçin Aksiyon Sınıfı**
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
