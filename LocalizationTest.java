package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class LocalizationTest extends LinearOpMode {
    private DcMotorEx rightElevator;
    private DcMotorEx leftElevator;
    private DcMotorEx YatayKolMotoru;
    private Servo YukariAsagiServo;
    private CRServo TekerlekServo;
    private Servo SagSolServo;
    private Servo KiskacServo;




    @Override
    public void runOpMode() throws InterruptedException {
        leftElevator = hardwareMap.get(DcMotorEx.class, "leftElevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rightElevator");
        YatayKolMotoru = hardwareMap.get(DcMotorEx.class, "YatayKolMotoru");
        YukariAsagiServo = hardwareMap.get(Servo.class, "YukariAsagiServo");
        TekerlekServo = hardwareMap.get(CRServo.class, "TekerlekServo");
        SagSolServo = hardwareMap.get(Servo.class, "SagSolServo");
        KiskacServo = hardwareMap.get(Servo.class, "KiskacServo");



        leftElevator.setDirection(DcMotorEx.Direction.FORWARD);
        rightElevator.setDirection(DcMotorEx.Direction.REVERSE);
        YatayKolMotoru.setDirection(DcMotorEx.Direction.REVERSE);


        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        YatayKolMotoru.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        YatayKolMotoru.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        YatayKolMotoru.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean holdPosition = true;

        double joystickInput = gamepad2.left_stick_x; // Joystick yukarı-aşağı kontrolü

        double currentPosition = YatayKolMotoru.getCurrentPosition();
        double position1 = 50.0;  // 50 derece
        double position2 = 150.0; // 150 derece
        double position3 = 250.0; // 250 derece
        TekerlekServo.setPower(0.0);  // Başlangıçta servoyu durdurmak için 0.0 (ya da istediğiniz pozisyon)
        double KolYukari= 0.263; ////Sayı Küçüldükçe Kol Yukarı Çıkıyor 0.265
        double KolAsagi= 0.845; //Sayı Büyüdükçe Kol Asağı İniyor
        double KolDuz= 0.700;
        double SagSolKolKapalı=0.260;
        double SagSolKolAcik=0.580;
        boolean YukariAsagiServoAclmaUygunMu = false;
        boolean YatayMotorAcılmaUygunMu= false;
        boolean motorAcik = false;
        boolean oncekiDurum = false;
        boolean kiskacKapali = false; // Kıskacın başlangıç durumu (açık)
        boolean kiskacOncekiDurum = false; // Önceki buton durumu



        double maxPosition = 250.0; // Motorun ulaşabileceği maksimum encoder pozisyonu
        double minPosition = 0.0;    // Motorun geri dönebileceği minimum encoder pozisyonu
        int threshold = 35;  // Motorun hedef pozisyona ne kadar yakın olursa duracağı tolerans
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if(gamepad1.dpad_up) {

                    leftElevator.setTargetPosition(1500);
                    rightElevator.setTargetPosition(1500);

                    leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                    leftElevator.setPower(0.5);
                    rightElevator.setPower(0.5);
                }

                if(gamepad1.dpad_down) {
                    leftElevator.setTargetPosition(0);
                    rightElevator.setTargetPosition(0);

                    leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftElevator.setPower(-0.5);
                    rightElevator.setPower(-0.5);
                }
                if (gamepad1.dpad_left)
                { leftElevator.setTargetPosition(1055);
                    rightElevator.setTargetPosition(1055);

                    leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftElevator.setPower(-0.3);
                    rightElevator.setPower(-0.3);}









                if (gamepad2.left_stick_x > 0 && YatayMotorAcılmaUygunMu) {
                    // Sağ yönde hareket
                    while (currentPosition <= maxPosition) {
                        currentPosition = YatayKolMotoru.getCurrentPosition(); // Güncelle
                        YatayKolMotoru.setPower(0.5);

                        // Gamepad durumu ya da pozisyon toleransı kontrolü
                        if (gamepad2.left_stick_x == 0 || Math.abs(currentPosition - maxPosition) < threshold) {
                            YatayKolMotoru.setPower(0); // Motoru durdur
                            break;
                        }
                    }
                } else if (gamepad2.left_stick_x < 0) {
                    // Sol yönde hareket
                    while (currentPosition >= minPosition) {
                        currentPosition = YatayKolMotoru.getCurrentPosition(); // Güncelle
                        YatayKolMotoru.setPower(-0.5);

                        // Gamepad durumu ya da pozisyon toleransı kontrolü
                        if (gamepad2.left_stick_x == 0 || Math.abs(currentPosition - maxPosition) < threshold) {
                            YatayKolMotoru.setPower(0); // Motoru durdur
                            break;
                        }
                    }
                } else {
                    YatayKolMotoru.setPower(0); // Motoru durdur
                }

                // Geri hareket sırasında, hedef pozisyonu geçme durumuna karşı iyileştirme
                if (gamepad2.left_stick_x < 0) {
                    if (Math.abs(currentPosition - minPosition) < threshold) {
                        YatayKolMotoru.setPower(0); // Motoru durdur
                    }
                }



                if (gamepad2.circle && YukariAsagiServoAclmaUygunMu) { // En Aşağı Konumda ()
                   YukariAsagiServo.setPosition(KolAsagi);
                    TekerlekServo.setPower(1.0);
                } else if (gamepad2.square) { //En Yukarı Konumda
                    YukariAsagiServo.setPosition(KolYukari);
                    TekerlekServo.setPower(0.0);

                    if (currentPosition >= 10) {
                        // Önce hedef pozisyonu belirle
                        YatayKolMotoru.setTargetPosition(0);

                        // Sonra RUN_TO_POSITION moduna geçir
                        YatayKolMotoru.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // Motoru hareket ettir
                        YatayKolMotoru.setPower(0.5);

                        // Motor hedefe ulaşana kadar bekle
                        while (opModeIsActive() && YatayKolMotoru.isBusy()) {
                            telemetry.addData("Motor Pozisyonu", YatayKolMotoru.getCurrentPosition());
                            telemetry.update();
                        }

                        // Motor durduğunda gücü sıfırla
                        YatayKolMotoru.setPower(0.0);

                        // Motor modunu güncelle (manuel kontrole geçmek için)
                        YatayKolMotoru.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
                else if (gamepad2.cross && YukariAsagiServoAclmaUygunMu) { //Tam Düz Konumda
                    YukariAsagiServo.setPosition(KolDuz);
                    TekerlekServo.setPower(0.0);}

                if (gamepad2.triangle && !oncekiDurum) {
                    motorAcik = !motorAcik; // Her basıldığında değişkeni tersine çevir
                    TekerlekServo.setPower(motorAcik ? -1.0 : 0.0); // Eğer açıksa -1.0, değilse 0.0
                }

                // Butonun önceki durumunu güncelle
                oncekiDurum = gamepad2.triangle;


                if (gamepad2.dpad_left) {
                    YukariAsagiServoAclmaUygunMu = false;
                    YatayMotorAcılmaUygunMu= false;

                    // Kol yukarıda değilse önce yukarı kaldır ve 1000ms bekle
                    if (YukariAsagiServo.getPosition() != KolYukari) {
                        YukariAsagiServo.setPosition(KolYukari);
                        TekerlekServo.setPower(0.0);
                        sleep(1000);
                    }

                    // Yatay kolun pozisyonu 10'dan büyükse sıfıra çek
                    if (currentPosition >= 10) {
                        // Önce hedef pozisyonu belirle
                        YatayKolMotoru.setTargetPosition(0);

                        // Sonra RUN_TO_POSITION moduna geçir
                        YatayKolMotoru.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // Motoru hareket ettir
                        YatayKolMotoru.setPower(0.5);

                        // Motor hedefe ulaşana kadar bekle
                        while (opModeIsActive() && YatayKolMotoru.isBusy()) {
                            telemetry.addData("Motor Pozisyonu", YatayKolMotoru.getCurrentPosition());
                            telemetry.update();
                        }

                        // Motor durduğunda gücü sıfırla
                        YatayKolMotoru.setPower(0.0);

                        // Motor modunu güncelle (manuel kontrole geçmek için)
                        YatayKolMotoru.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    // Sağ-Sol servoyu kapalı konuma getir
                    SagSolServo.setPosition(SagSolKolKapalı);
                }
                else if (gamepad2.dpad_right) {
                    SagSolServo.setPosition(SagSolKolAcik);
                    YukariAsagiServoAclmaUygunMu =true;
                    YatayMotorAcılmaUygunMu=true;

                }


                if (gamepad1.triangle && !kiskacOncekiDurum) {
                    kiskacKapali = !kiskacKapali; // Her basışta durumu tersine çevir
                    KiskacServo.setPosition(kiskacKapali ? 1 : 0); // Kapalıysa 1.0, açıksa 0.0
                }

                // Butonun önceki durumunu güncelle
                kiskacOncekiDurum = gamepad1.triangle;





                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                // Elevator Encoder Telemetry
                telemetry.addData("Left Elevator Encoder", leftElevator.getCurrentPosition());
                telemetry.addData("Right Elevator Encoder", rightElevator.getCurrentPosition());
                telemetry.addData("Yatay Elevator Encoder", YatayKolMotoru.getCurrentPosition());
                telemetry.addData("Gamepad2 Left X", gamepad2.left_stick_x);
                telemetry.addData("Gamepad2 Left Y", gamepad2.left_stick_y);
                telemetry.addData("YukariAsagiServo Verisi", YukariAsagiServo.getPosition());
                telemetry.addData("SagSolServo Verisi", SagSolServo.getPosition());
                telemetry.addData("TekerlekServo Verisi", TekerlekServo.getPower());
                telemetry.addData("YukariAsagiServoAclmaUygunMu", YukariAsagiServoAclmaUygunMu);
                telemetry.addData("YatayMotorAcılmaUygunMu ", YatayMotorAcılmaUygunMu);
                telemetry.addData("Kıskaç Servo Değer", KiskacServo.getPosition());





                telemetry.update();

                // FTCDashboard için Telemetry Packet
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);


            }
        } else {
            throw new RuntimeException();
        }
    }
}
