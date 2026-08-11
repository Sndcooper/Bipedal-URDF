from biped_rviz_launch_suite.launch_helpers import RobotModelSpec, build_standard_display_launch_description


def generate_launch_description():
    return build_standard_display_launch_description(
        RobotModelSpec(
            asset_package='URDF_description',
            model_relpath='urdf/URDF.xacro',
            model_kind='xacro',
            include_replacements=(
                ('$(find URDF_description)/urdf/', ''),
            ),
            mesh_source_prefixes=(
                'file://$(find URDF_description)/meshes/',
            ),
        )
    )