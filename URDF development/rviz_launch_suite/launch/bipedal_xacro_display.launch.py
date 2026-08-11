from biped_rviz_launch_suite.launch_helpers import RobotModelSpec, build_standard_display_launch_description


def generate_launch_description():
    return build_standard_display_launch_description(
        RobotModelSpec(
            asset_package='bipedal_description',
            model_relpath='urdf/bipedal.xacro',
            model_kind='xacro',
            include_replacements=(
                ('$(find Bipedal_description)/urdf/', ''),
            ),
            mesh_source_prefixes=(
                'file://$(find Bipedal_description)/meshes/',
            ),
        )
    )