from biped_rviz_launch_suite.launch_helpers import RobotModelSpec, build_assembly_display_launch_description


def generate_launch_description():
    return build_assembly_display_launch_description(
        RobotModelSpec(
            asset_package='assembly_description',
            model_relpath='urdf/assembly.xacro',
            model_kind='xacro',
            include_replacements=(
                ('$(find assembly_description)/urdf/', ''),
            ),
            mesh_source_prefixes=(
                'file://$(find assembly_description)/meshes/',
            ),
        )
    )