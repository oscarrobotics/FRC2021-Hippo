package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class ComplexAutonomous extends SequentialCommandGroup {

    public ComplexAutonomous(DrivetrainSubsystem drivetrain) {

        var endPose = drivetrain.getLatestPose().transformBy(new Transform2d(new Translation2d(1,0), Rotation2d.fromDegrees(0)));
        addCommands(
                new Navigate(endPose, drivetrain)
        );
    }
}
