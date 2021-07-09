package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import static frc.team832.robot.Constants.ShooterValues.*;

public class VisionSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private final PhotonCamera gloworm = new PhotonCamera("gloworm");

    private NetworkTableEntry dashboard_visionYaw, dashboard_visionPitch, dashboard_distance, dashboard_exitAngle;

    private PhotonTrackedTarget target;
    private boolean hasTargets;
    private double exitAngle;

    public VisionSubsystem() {
        setName("Vision");
        DashboardManager.addTab(this);

        dashboard_visionYaw = DashboardManager.addTabItem(this, "VisionYaw", 0.0);
        dashboard_visionPitch = DashboardManager.addTabItem(this, "VisionPitch", 0.0);
        dashboard_distance = DashboardManager.addTabItem(this, "VisionDistance", 0.0);
        dashboard_exitAngle = DashboardManager.addTabItem(this, "ExitAngle", 0.0);
        initSuccessful = true;
    }

    @Override
    public void periodic(){
        updateVision();
    }

    private void updateVision() {
        PhotonPipelineResult latestResult = gloworm.getLatestResult();
        hasTargets = latestResult.hasTargets();
        if (hasTargets) {
            target = latestResult.getBestTarget();
            updateShooterCalculations(target.getPitch(), target.getYaw());
        }
    }

    private void updateShooterCalculations(double pitch, double yaw) {
        double distance = PhotonUtils.calculateDistanceToTargetMeters(CameraHeightMeters, PowerportHeightMeters, Math.toRadians(CameraAngle), Math.toRadians(pitch));
        double angle = (distance * 0.2896) + 42.803;
        exitAngle = angle;

        dashboard_visionYaw.setDouble(yaw);
        dashboard_visionPitch.setDouble(pitch);
        dashboard_distance.setDouble(distance);
        dashboard_exitAngle.setDouble(exitAngle);
    }

    public PhotonTrackedTarget getTarget() {
        return target;
    }

    public void driverMode(boolean enable) {
        gloworm.setDriverMode(enable);
    }

    public boolean hasTarget() {
        return hasTargets;
    }

    public double getSmartHoodAngle() {
        return exitAngle;
    }

//    public static double getSpindexerRpm(){
//        return Math.pow(0.5, (0.4 * distance) - 5) + 90;
//    }
}


