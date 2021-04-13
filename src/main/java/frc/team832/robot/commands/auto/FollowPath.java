package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.OscarRamseteCommand;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class FollowPath extends OscarRamseteCommand {
    private static final RamseteController ramseteController = new RamseteController();
    private static final PIDController leftDrivePIDController = new PIDController(Constants.DrivetrainValues.LeftConfig.getkP(), 0, 0);
    private static final PIDController rightDrivePIDController = new PIDController(Constants.DrivetrainValues.RightConfig.getkP(), 0, 0);

    public FollowPath(Trajectory trajectory, DrivetrainSubsystem drivetrain) {
        super(trajectory, drivetrain::getLatestPose, ramseteController, Constants.DrivetrainValues.LeftFF,
                Constants.DrivetrainValues.RightFF, Constants.DrivetrainValues.DriveKinematics,
                drivetrain::getWheelSpeeds, leftDrivePIDController, rightDrivePIDController,
                drivetrain::setWheelVolts, drivetrain);
        addRequirements(drivetrain);
    }
}
