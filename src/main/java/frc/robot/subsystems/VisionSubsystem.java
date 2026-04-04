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

    /**
     * YENİ API yapısına uygun Kamera Güncelleme Metodu
     */
    private void updatePhotonCamera(PhotonPoseEstimator estimator, PhotonCamera camera) {
        // 1. Kameradan o anki çerçeveyi (sonucu) çek
        var result = camera.getLatestResult();

        // 2. Çerçevede herhangi bir hedef var mı?
        if (result.hasTargets()) {
            // 3. Stratejiyi direkt metot ismi olarak çağırıyoruz (Coprocessor'da Multi-Tag)
            Optional<EstimatedRobotPose> estPose = estimator.estimateCoprocMultiTagPose(result);
            
            // Güvenlik (Fallback): Eğer sadece 1 etiket varsa MultiTag başarısız olur, 
            // o yüzden tekli etiket stratejisi olan "Lowest Ambiguity" ile tekrar deneriz.
            if (estPose.isEmpty()) {
                estPose = estimator.estimateLowestAmbiguityPose(result);
            }

            // 4. Eğer elimizde geçerli bir konum tahmini varsa şaseye gönder
            estPose.ifPresent(estimatedRobotPose -> {
                Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
                double timestamp = estimatedRobotPose.timestampSeconds;
                
                drivetrain.addVisionMeasurement(pose, timestamp);
            });
        }
    }
}