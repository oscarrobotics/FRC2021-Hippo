package frc.team832.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.FalconDashboard;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.RobotContainer;
import frc.team832.robot.utilities.ArcadeDriveProfile;
import frc.team832.robot.utilities.TankDriveProfile;

public class DrivetrainSubsystem extends SubsystemBase {

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

    private final double loopPeriod = 0.005;

//    private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
//            Nat.N1(), Nat.N1(),
//            Constants.DrivetrainValues.kDrivetrainPlant,
//            VecBuilder.fill(3.0), // How accurate we think our model is
//            VecBuilder.fill(0.01), // How accurate we think our encoder data is
//            loopPeriod);
//
//    private final LinearQuadraticRegulator<N1, N1, N1> m_controller
//            = new LinearQuadraticRegulator<>(Constants.DrivetrainValues.kDrivetrainPlant,
//            VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
//            // this to more heavily penalize state excursion, or make the controller behave more aggressively.
//            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
//            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
//            // starting point because that is the (approximate) maximum voltage of a battery.
//            loopPeriod); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.
//
//    private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
//            Constants.DrivetrainValues.kDrivetrainPlant,
//            m_controller,
//            m_observer,
//            12.0,
//            loopPeriod);


    private final TankDriveProfile tankProfile;
    private final ArcadeDriveProfile arcadeProfile;

    private final SmartMCAttachedPDPSlot leftMasterSlot, leftSlaveSlot, rightMasterSlot, rightSlaveSlot;
    private final NetworkTableEntry dashboard_rightVolts, dashboard_leftVolts, dashboard_pigeonIMU_pitch, dashboard_pigeonIMU_roll, dashboard_pigeonIMU_fusedHeading,
            dashboard_poseX, dashboard_poseY, dashboard_poseRotation, dashboard_rawLeftPos, ui_poseX, ui_poseY;


    public DrivetrainSubsystem(GrouchPDP pdp, DriverOI oi) {
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

        tankProfile = new TankDriveProfile(oi, false,false);
        arcadeProfile = new ArcadeDriveProfile(oi);

        var defaultStartPose = Constants.FieldPosition.InitLine_CenteredOnPort;

        // startPoseChooser
        startPoseChooser.addOption(defaultStartPose.toString(), defaultStartPose.poseMeters);
        startPoseChooser.setDefaultOption(defaultStartPose.toString(), defaultStartPose.poseMeters);

        startPoseChooser.addOption(Constants.FieldPosition.ZeroZero.toString(), Constants.FieldPosition.ZeroZero.poseMeters);

        DashboardManager.addTab(this);
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
//        DashboardManager.getTab(this).add("ResetPose", dashboardResetPoseCommand);

        startingPose = defaultStartPose.poseMeters;
        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, Constants.DrivetrainValues.ClosedLoopDT, Constants.DrivetrainValues.MaxRpm);
        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);
        resetPose();

        setDefaultCommand(new RunEndCommand(this::tankDrive, this::stopDrive, this));

        initSuccessful = leftMaster.getCANConnection() && leftSlave.getCANConnection() &&
                rightMaster.getCANConnection() && rightSlave.getCANConnection() && imu.getState() != PigeonIMU.PigeonState.NoComm;
    }

    @Override
    public void periodic() {
        updatePose();
        startingPose = startPoseChooser.getSelected();

        FalconDashboard.updateRobotPose2d(robotPose);
        imu.getYawPitchRoll(ypr);
        dashboard_leftVolts.setDouble(leftMaster.getOutputVoltage());
        dashboard_rightVolts.setDouble(rightMaster.getOutputVoltage());
        dashboard_pigeonIMU_pitch.setDouble(ypr[1]);
        dashboard_pigeonIMU_roll.setDouble(ypr[2]);
        dashboard_pigeonIMU_fusedHeading.setDouble(imu.getFusedHeading());
        dashboard_poseX.setDouble(robotPose.getTranslation().getX());
        dashboard_poseY.setDouble(robotPose.getTranslation().getY());
        dashboard_poseRotation.setDouble(robotPose.getRotation().getDegrees());
        dashboard_rawLeftPos.setDouble(leftMaster.getSensorPosition());
        ui_poseX.setDouble(startingPose.getTranslation().getX());
        ui_poseY.setDouble(startingPose.getTranslation().getY());
    }

    private void tankDrive() {
        tankProfile.calculateTankSpeeds();
        diffDrive.tankDrive(tankProfile.leftPower, tankProfile.rightPower, tankProfile.loopMode);
    }

    @SuppressWarnings("unused")
    public void arcadeDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    @SuppressWarnings("unused")
    public void xBoxDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    private void stopDrive() {
        leftMaster.set(0);
        rightMaster.set(0);
    }

    private Rotation2d getDriveHeading() {
        double trimmedHeading = OscarMath.round(imu.getFusedHeading(), 3);
        return Rotation2d.fromDegrees(trimmedHeading);
    }

    private double getRightDistanceMeters() {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(rightMaster.getSensorPosition());
    }

    private double getLeftDistanceMeters() {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(-leftMaster.getSensorPosition());
    }

    private double getRightVelocityMetersPerSec() {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(rightMaster.getSensorVelocity());
    }

    private double getLeftVelocityMetersPerSec() {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(leftMaster.getSensorVelocity());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    public void setWheelVolts(Double leftVolts, Double rightVolts) {
        leftMaster.set(leftVolts);
        rightMaster.set(rightVolts);
    }

    public Pose2d getLatestPose() {
        updatePose();
        return robotPose;
    }

    private void updatePose() {
        robotPose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    public void resetPose(double x, double y) {
        resetPose(new Pose2d(x, y, getDriveHeading()));
    }

    private void resetPose(Pose2d pose) {
        resetEncoders();
        zeroYaw();
        robotPose = pose;
        driveOdometry.resetPosition(robotPose, getDriveHeading());
    }

    private void resetPoseFromDashboard() {
        Pose2d pose = new Pose2d(ui_poseX.getDouble(startingPose.getTranslation().getX()), ui_poseY.getDouble(startingPose.getTranslation().getY()), getDriveHeading());
        resetPose(pose);
    }

    private void resetPose() {
        resetPose(startingPose);
    }

    private void resetEncoders() {
        leftMaster.rezeroSensor();
        rightMaster.rezeroSensor();
    }

    private void zeroYaw() {
        imu.setFusedHeading(startingPose.getRotation().getDegrees());
    }

    public void setNeutralMode(NeutralMode mode) {
        leftMaster.setNeutralMode(mode);
        leftSlave.setNeutralMode(mode);
        rightMaster.setNeutralMode(mode);
        rightSlave.setNeutralMode(mode);
    }

    private void setCurrentLimit(int amps) {
        leftMaster.limitInputCurrent(amps);
        leftSlave.limitInputCurrent(amps);
        rightMaster.limitInputCurrent(amps);
        rightSlave.limitInputCurrent(amps);
    }

    public void updateControlLoops() {
        robotPose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }
}
