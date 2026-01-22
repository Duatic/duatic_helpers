from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def simulator_not_subcomponent_condition(simulator: str):
    return IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration("start_as_subcomponent"),
            "'.lower() != 'true' and '",
            LaunchConfiguration("simulator"),
            f"' == '{simulator}'",
        ])
    )
