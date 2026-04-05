package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose; // Yeni eklendi
import edu.wpi.first.math.VecBuilder;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;

    // --- PhotonVision Kameraları ---
    private final PhotonCamera cam1 = new PhotonCamera("PhotonCam1");
    private final PhotonCamera cam2 = new PhotonCamera("PhotonCam2");
    private final PhotonCamera cam3 = new PhotonCamera("PhotonCam3");

    // Kameraların robotun merkezine göre konumları
    private final Transform3d robotToCam1 = new Transform3d(new Translation3d(0.2, 0.2, 0.5), new Rotation3d(0, 0, 0));
    private final Transform3d robotToCam2 = new Transform3d(new Translation3d(-0.2, 0.2, 0.5), new Rotation3d(0, 0, Math.PI)); 
    private final Transform3d robotToCam3 = new Transform3d(new Translation3d(0.0, -0.2, 0.5), new Rotation3d(0, 0, -Math.PI / 2)); 

    // Her kamera için ayrı konum tahminleyicisi
    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;
    private final PhotonPoseEstimator estimator3;

    // AdvantageScope için saha nesnesi
    private final Field2d field = new Field2d();
    
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putData("Field", field);

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile); 
        } catch (Exception e) {
            throw new RuntimeException("AprilTag haritası yüklenemedi!", e);
        } 

        estimator1 = new PhotonPoseEstimator(fieldLayout, robotToCam1);
        estimator2 = new PhotonPoseEstimator(fieldLayout, robotToCam2);
        estimator3 = new PhotonPoseEstimator(fieldLayout, robotToCam3);
    }

    @Override
    public void periodic() {
        // Artık fonksiyona hem estimator'ı hem de kameranın kendisini yolluyoruz
        updatePhotonCamera(estimator1, cam1);
        updatePhotonCamera(estimator2, cam2);
        updatePhotonCamera(estimator3, cam3);

        // Odometriyi ekrana bas
        field.setRobotPose(drivetrain.getState().Pose);
    }

private void updatePhotonCamera(PhotonPoseEstimator estimator, PhotonCamera camera) {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            Optional<EstimatedRobotPose> estPose = estimator.estimateCoprocMultiTagPose(result);
            
            if (estPose.isEmpty()) {
                estPose = estimator.estimateLowestAmbiguityPose(result);
            }

            estPose.ifPresent(estimatedRobotPose -> {
                Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
                double timestamp = estimatedRobotPose.timestampSeconds;
                
                // 1. Görülen hedeflerin robota olan ortalama uzaklığını hesapla
                double avgDistance = 0;
                for (var target : result.getTargets()) {
                    avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgDistance /= result.getTargets().size();

                // 2. Güven (Sapma) Çarpanlarını Belirle (Değerler büyüdükçe güven AZALIR)
                double xyStdDev;
                double thetaStdDev;

                // Eğer birden fazla etiket görüyorsa (Multi-Tag), sistem çok daha güvenilirdir
                if (result.getTargets().size() > 1) {
                    // Uzaklığın karesiyle orantılı olarak sapmayı artır (örn: yakında 0.1, uzakta 0.5+)
                    xyStdDev = 0.1 + (Math.pow(avgDistance, 2) * 0.05); 
                    thetaStdDev = 0.1 + (Math.pow(avgDistance, 2) * 0.05);
                } 
                // Eğer tek etiket görüyorsa güveni ciddi şekilde düşür
                else {
                    // Güvenlik Önlemi: Tek etiket 4 metreden uzaksa bu veriyi tamamen reddet (Noise'u engeller)
                    if (avgDistance > 4.0) {
                        return; 
                    }
                    xyStdDev = 0.3 + (Math.pow(avgDistance, 2) * 0.1); 
                    thetaStdDev = 0.5 + (Math.pow(avgDistance, 2) * 0.2);
                }

                // 3. Standart Sapma Matrisini Oluştur (X Metre, Y Metre, Radyan)
                var visionMeasurementStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

                // 4. Veriyi ve GÜVEN SKORUNU Swerve şaseye gönder
                drivetrain.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
            });
        }
    }
}