from biped_rviz_launch_suite.launch_helpers import RobotModelSpec, build_standard_display_launch_description


def generate_launch_description():
    return build_standard_display_launch_description(
        RobotModelSpec(
            asset_package='bipedal_export_temp_description',
            model_relpath='urdf/bipedal_export_temp_visual.urdf',
            model_kind='urdf',
            mesh_final_prefixes=(
                '../meshes/',
            ),
        )
    )