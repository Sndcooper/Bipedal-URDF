from biped_rviz_launch_suite.launch_helpers import RobotModelSpec, build_standard_display_launch_description


def generate_launch_description():
    return build_standard_display_launch_description(
        RobotModelSpec(
            asset_package='bipedal_description',
            model_relpath='urdf/bipedal.urdf.xacro',
            model_kind='xacro',
            mesh_asset_package='bipedal_export_temp_description',
            mesh_source_prefixes=(
                'package://bipedal_export_temp_description/meshes/',
            ),
        )
    )