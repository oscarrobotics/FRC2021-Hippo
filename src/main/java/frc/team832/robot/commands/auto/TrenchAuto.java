// package frc.team832.robot.commands.auto;

// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Transform2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.team832.robot.subsystems.DrivetrainSubsystem;

// public class TrenchAuto extends SequentialCommandGroup {

//     public TrenchAuto(DrivetrainSubsystem drivetrain) {

// //        Trajectory ToStart = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPosition.StartCenter.poseMeters);
// //		Trajectory ToCloseSideTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPosition.CloseSideTrench.poseMeters);
// //		Trajectory ToFarSideTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPosition.FarSideTrench.poseMeters);
// //		Trajectory ToShieldGenCloseToTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPosition.ShieldGenCloseToTrench.poseMeters);

//         addCommands(
// //			new ShootCommandGroup(superStructure),
// //			new FollowPath(ToFarSideTrench, drivetrain),
// //			new InstantCommand(superStructure::intake),
// //			new FollowPath(ToCloseSideTrench, drivetrain),
// //			new InstantCommand(superStructure::idleIntake),
// //			new FollowPath(ToFarSideTrench, drivetrain),
// //			new ShootCommandGroup(superStructure),
// //			new InstantCommand(superStructure::intake),
// //			new FollowPath(ToShieldGenCloseToTrench, drivetrain),
// //			new InstantCommand(superStructure::idleIntake),
// //			new FollowPath(ToFarSideTrench, drivetrain),
// //			new ShootCommandGroup(superStructure)
//         );
//         addRequirements(superStructure, drivetrain, shooter, spindexer);


//     }
// }
