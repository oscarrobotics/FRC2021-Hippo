package frc.team832.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.robot.Constants;
//import frc.team832.robot.utilites.TankDriveProfile;

public class Drivetrain extends SubsystemBase implements DashboardUpdatable {
    public final boolean initSuccessful;

    private final CANTalonFX leftMaster, leftSlave, rightMaster, rightSlave;

    private final PigeonIMU imu;
    private final double[] ypr = new double[3];

    private final SmartDiffDrive diffDrive;
    private final DifferentialDriveOdometry driveOdometry;
    private final SendableChooser<Pose2d> startPoseChooser = new SendableChooser<>();

    private Pose2d startingPose = new Pose2d();
    private Pose2d robotPose = startingPose;

    private double latestLeftWheelVolts, latestRightWheelVolts;

//    private final TankDriveProfile tankProfile = new TankDriveProfile(false, false);
//    private final ArcadeDriveProfile arcadeProfile = new ArcadeDriveProfile();

    private final SmartMCAttachedPDPSlot leftMasterSlot, leftSlaveSlot, rightMasterSlot, rightSlaveSlot;
    private final NetworkTableEntry dashboard_rightVolts, dashboard_leftVolts, dashboard_pigeonIMU_pitch, dashboard_pigeonIMU_roll, dashboard_pigeonIMU_fusedHeading,
            dashboard_poseX, dashboard_poseY, dashboard_poseRotation, dashboard_rawLeftPos, ui_poseX, ui_poseY;

/*
*   private final CommandBase dashboardResetPoseCommand = ;
*
*
*/

    public Drivetrain(GrouchPDP pdp) {
        leftMaster = new CANTalonFX(Constants.DrivetrainValues.LEFT_MASTER_CAN_ID);
        leftSlave = new CANTalonFX(Constants.DrivetrainValues.LEFT_SLAVE_CAN_ID);
        rightMaster = new CANTalonFX(Constants.DrivetrainValues.RIGHT_MASTER_CAN_ID);
        rightSlave = new CANTalonFX(Constants.DrivetrainValues.RIGHT_SLAVE_CAN_ID);

        leftMaster.wipeSettings();
        leftSlave.wipeSettings();
        rightMaster.wipeSettings();
        rightSlave.wipeSettings();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        leftSlave.setInverted(true);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        leftMasterSlot = pdp.addDevice(Constants.DrivetrainValues.LEFT_MASTER_PDP_PORT, leftMaster);
        leftSlaveSlot = pdp.addDevice(Constants.DrivetrainValues.LEFT_SLAVE_PDP_PORT, leftSlave);
        rightMasterSlot = pdp.addDevice(Constants.DrivetrainValues.RIGHT_MASTER_PDP_PORT, rightMaster);
        rightSlaveSlot = pdp.addDevice(Constants.DrivetrainValues.RIGHT_SLAVE_PDP_PORT, rightSlave);

        setNeutralMode(NeutralMode.kBrake);
        setCurrentLimit(40);
        Constants.DrivetrainValues.ClosedLoopDT.setFFAccel(0.1);

        imu = new PigeonIMU(0);

        var defaultStartPose = Constants.FieldPosition.InitLine_CenteredOnPort;

        // startPoseChooser
        startPoseChooser.addOption(defaultStartPose.toString(), defaultStartPose.poseMeters);
        startPoseChooser.setDefaultOption(defaultStartPose.toString(), defaultStartPose.poseMeters);

        startPoseChooser.addOption(Constants.FieldPosition.ZeroZero.toString(), Constants.FieldPosition.ZeroZero.poseMeters);

        DashboardManager.addTab(this, this);
        dashboard_rightVolts = DashboardManager.addTabItem(this, "Raw/RightVolts", 0.0);
        dashboard_leftVolts = DashboardManager.addTabItem(this, "Raw/LeftVolts", 0.0);
        dashboard_pigeonIMU_pitch = DashboardManager.addTabItem(this, "IMU/Pitch", 0.0);
        dashboard_pigeonIMU_roll = DashboardManager.addTabItem(this, "IMU/Roll", 0.0);
        dashboard_pigeonIMU_fusedHeading = DashboardManager.addTabItem(this, "IMU/FusedHeading", 0.0);
        dashboard_poseX = DashboardManager.addTabItem(this, "Pose/X", 0.0);
        dashboard_poseY = DashboardManager.addTabItem(this, "Pose/Y", 0.0);
        dashboard_poseRotation = DashboardManager.addTabItem(this, "Pose/Rotation", 0.0);
        dashboard_rawLeftPos = DashboardManager.addTabItem(this, "Raw/LeftPos", 0.0);
        ui_poseX = DashboardManager.addTabItem(this, "Starting Pose X", startingPose.getTranslation().getX());
        ui_poseY = DashboardManager.addTabItem(this, "Starting Pose Y", startingPose.getTranslation().getY());
        DashboardManager.getTab(this).add("StartPose", startPoseChooser);
        DashboardManager.getTab(this).add("ResetPose", dashboardResetPoseCommand);

        startingPose = defaultStartPose.poseMeters;
        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, Constants.DrivetrainValues.ClosedLoopDT, Constants.DrivetrainValues.MaxRpm);
        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);
        resetPose();

        setDefaultCommand(new RunEndCommand(this::tankDrive, this::stopDrive, this));

        initSuccessful = leftMaster.getCANConnection() && leftSlave.getCANConnection() &&
                rightMaster.getCANConnection() && rightSlave.getCANConnection() && imu.getState() != PigeonIMU.PigeonState.NoComm;
    }

    @Override
    public String getDashboardTabName() {
        return null;
    }

    @Override
    public void updateDashboardData() {

    }
}
